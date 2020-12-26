//#define LOG_NDEBUG 0
#define LOG_TAG "Xsy_CmdControl_JNI"


#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <limits.h>
#include <unistd.h>
#include <fcntl.h>
#include <android/log.h>
#include "MyUtils.h"
#include <utils/RefBase.h>
#include <utils/StrongPointer.h>
//#include <android_runtime/AndroidRuntime.h>
#include <utils/Mutex.h>
#include <nativehelper/JNIHelp.h>
#include <jni.h>
#include "SerialCommand.h"
using namespace android;
namespace cmdcontrol {
#ifdef __cplusplus
extern "C" {
#endif

/* get #of elements in a static array */
#ifndef NELEM
# define NELEM(x) ((int) (sizeof(x) / sizeof((x)[0])))
#endif

static const char* XSY_CMDCONTROL_JNI_CLASS_PATH_NAME =
		"com/xinshiyun/telephoneserver/widget/CmdControlJNI";

static Mutex sLock;
static JavaVM *gs_jvm = NULL;
static JNIEnv* mEnv = NULL;

struct fields_t {
	jfieldID context;
	jmethodID postEvent;
};
static fields_t fields;

class JNICmdControlListener: public CmdControlListener {
public:
	JNICmdControlListener(JNIEnv* env, jobject thiz, jobject weak_thiz);
	~JNICmdControlListener();
	virtual void notify(int what, int ext1, char* ext2) ;
private:
	JNICmdControlListener();
	jclass mClass; // Reference to MediaPlayer class
	jobject mObject; // Weak ref to MediaPlayer Java object to call on
};

//JNICmdControlListener Implements
JNICmdControlListener::JNICmdControlListener(JNIEnv* env, jobject thiz,
		jobject weak_thiz) {
	LOGD("Construct JNICmdControlListener");
	// Hold onto the SoundbarDisplayControl class for use in calling the static method
	// that posts events to the application thread.
	jclass clazz = env->GetObjectClass(thiz);
	if (clazz == NULL) {
		LOGE("Can't find com/xinshiyun/telephoneserver/widget/CmdControlJNI");
		//jniThrowException(env, "java/lang/Exception", NULL);
		return;
	}
	mClass = (jclass) env->NewGlobalRef(clazz);

	// We use a weak reference so the SoundbarDisplayControl object can be garbage collected.
	// The reference is only used as a proxy for callbacks.
	mObject = env->NewGlobalRef(weak_thiz);
}

JNICmdControlListener::~JNICmdControlListener() {
	// remove global references
	LOGD("Release JNICmdControlListener");
	//JNIEnv *env = AndroidRuntime::getJNIEnv();
	mEnv->DeleteGlobalRef(mObject);
	mEnv->DeleteGlobalRef(mClass);
}

void JNICmdControlListener::notify(int what, int ext1, char* ext2) {
	//what-->ser_cmd, ext1-->ser_data,ext2-->ser_mutedata,obj-->NULL
	//将ser_data unsigned char 转为int 传给java层处理
	//不能关联到当前JVM的中的线程，包Fatal signal 11 错误
	//JNIEnv *env = AndroidRuntime::getJNIEnv();
	int status;
	JNIEnv *env;
	bool isAttached = false;
	status = gs_jvm->GetEnv((void**) &env, JNI_VERSION_1_4);
	if (status < 0) {
		gs_jvm->AttachCurrentThread(&env, NULL); //将当前线程注册到虚拟机中．
		isAttached = true;
	}

	//LOGD("what = 0x%x, ext1 = 0x%x, ext2 = %s",what,ext1,ext2);
	jstring result = env->NewStringUTF(ext2);
	env->CallStaticVoidMethod(mClass, fields.postEvent, mObject, what, ext1,result);

	if (env->ExceptionCheck()) {
		LOGW("An exception occurred while notifying an event.");
		env->ExceptionClear();
	}

	if (isAttached)
		gs_jvm->DetachCurrentThread();
}

static sp<SerialCommand> setCmdControl(JNIEnv* env, jobject thiz,
		const sp<SerialCommand>& SerCmd) {
	Mutex::Autolock l(sLock);
	sp<SerialCommand> old = (SerialCommand*) env->GetLongField(thiz,
			fields.context);
	if (SerCmd.get()) {
		SerCmd->incStrong((void*) setCmdControl);
	}
	if (old != 0) {
		old->decStrong((void*) setCmdControl);
	}
	env->SetLongField(thiz, fields.context, (jlong) SerCmd.get());
	return old;
}

static sp<SerialCommand> getCmdControl(JNIEnv* env, jobject thiz) {
	Mutex::Autolock l(sLock);
	SerialCommand* const p = (SerialCommand*) env->GetLongField(thiz,
			fields.context);
	return sp < SerialCommand > (p);
}

static void Xsy_CmdControl_native_init(JNIEnv *env) {
	jclass clazz;
	LOGD("native_init()");	
	clazz = env->FindClass(XSY_CMDCONTROL_JNI_CLASS_PATH_NAME);
	if (clazz == NULL) {
		return;
	}

	fields.context = env->GetFieldID(clazz, "mNativeContext", "J");
	if (fields.context == NULL) {
		return;
	}

	fields.postEvent = env->GetStaticMethodID(clazz, "postEventFromNative","(Ljava/lang/Object;IILjava/lang/String;)V");
	if (fields.postEvent == NULL) {
		return;
	}

	env->DeleteLocalRef(clazz);
}

static void Xsy_CmdControl_native_setup(JNIEnv *env,
		jobject thiz, jobject weak_this) {
	LOGD("native_setup()");
	sp<SerialCommand> SerCom = new SerialCommand();
	if (SerCom == NULL) {
		//jniThrowException(env, "java/lang/RuntimeException", "Out of memory");
		return;
	}

	// create new listener and give it to SerialCommand
	sp<JNICmdControlListener> listener = new JNICmdControlListener(env, thiz,
			weak_this);
	SerCom->setListener(listener);
	SerCom->iniSer();
	// Stow our new C++ SerialCommand in an opaque field in the Java object.
	setCmdControl(env, thiz, SerCom);
}

static void Xsy_CmdControl_native_finalize(JNIEnv *env,
		jobject thiz) {
	LOGD("native_finalize()");
	sp<SerialCommand> serCmd = getCmdControl(env, thiz);
	serCmd->deInitSer();
	serCmd->setListener(0);
	if (serCmd != NULL) {
		setCmdControl(env, thiz, 0);
		LOGD("native_finalize() exit");
		/*sp<SerialCommand> oldSerCmd = setCmdControl(env, thiz, 0);
		if (oldSerCmd != NULL) {
			// this prevents native callbacks after the object is released
			LOGD("native_finalize() set old SerCmd null");
			oldSerCmd->deInitSer();
			oldSerCmd->setListener(0);
		}*/
	}
}

static jint Xsy_CmdControl_onExecuteCmd(JNIEnv *env,
		jobject thiz, jstring cmd){
	LOGD("onExecuteCmd()");
	jint result = -1;
	sp<SerialCommand> serCom = getCmdControl(env, thiz);
	if (serCom != NULL) {
		if(cmd != NULL){
			const char* command = env->GetStringUTFChars(cmd,NULL);
			result = serCom->onExecuteCmd(command);
			env->ReleaseStringUTFChars(cmd,command);
			return result;
		}
	}
	return JNI_OK;
}

static jint Xsy_CmdControl_onProcessPCMData(JNIEnv *env,
		jobject thiz,jbyteArray pcmdata,jint len){
	LOGD("onProcssPCMData()");
	jint result = -1;
	sp<SerialCommand> serCom = getCmdControl(env, thiz);
	if (serCom != NULL && len != 0) {
		jbyte* pcmData = env->GetByteArrayElements(pcmdata,NULL);
		jint pcm_len = env->GetArrayLength(pcmdata);
		if(pcm_len == len){
			unsigned char * tempPcmData = (unsigned char *)malloc(sizeof(unsigned char) * len);
			memcpy(tempPcmData,pcmData,len);
			result = serCom->onProcessPCMData(tempPcmData,len);
			free(tempPcmData);
			tempPcmData = NULL;
		}
		env->ReleaseByteArrayElements(pcmdata,pcmData,0);
		return result;
	}
	return JNI_OK;
}


static JNINativeMethod gCmdControlMethods[] =
{ 
	{"native_init", "()V", (void*) Xsy_CmdControl_native_init }, 
	{"native_setup", "(Ljava/lang/Object;)V",(void*) Xsy_CmdControl_native_setup }, 
	{"native_finalize", "()V",(void*) Xsy_CmdControl_native_finalize },
	{"onExecuteCmd", "(Ljava/lang/String;)I",(void*) Xsy_CmdControl_onExecuteCmd },
	{"onProcessPCMData", "([BI)I",(void*) Xsy_CmdControl_onProcessPCMData }
};

// This function only registers the native methods
static int register_android_cmd_control(JNIEnv *env) {
	LOGD("register_android_cmd_control()");
	return jniRegisterNativeMethods(env,
			XSY_CMDCONTROL_JNI_CLASS_PATH_NAME,
			gCmdControlMethods, NELEM(gCmdControlMethods));
}

jint JNI_OnLoad(JavaVM* vm, void* /* reserved */) {
	LOGD("JNI_OnLoad()");
	gs_jvm = vm;
	JNIEnv* env = 0;
	jint result = -1;

	if (vm->GetEnv((void**) &env, JNI_VERSION_1_4) != JNI_OK) {
		LOGD("JNI_OnLoad(): vm->getEnv() failed!!");
		goto bail;
	}

	mEnv = env;
	if (register_android_cmd_control(env) != JNI_OK) {
		LOGD("JNI_OnLoad(): call register_android_cmd_control() failed!!");
		goto bail;
	}

	/* success -- return valid version number */
	result = JNI_VERSION_1_4;

	bail: return result;
}

#ifdef __cplusplus
}
#endif

}
