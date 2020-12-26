/*
 * Utils.h
 *
 *  Created on: 2017-11-29
 *      Author: Administrator
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <jni.h>
#include <utils/RefBase.h>
//#include <binder/Parcel.h>
#include <android/log.h>
using namespace android;
namespace cmdcontrol {
#ifdef __cplusplus
extern "C" {
#endif

#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define LOGV(...)  __android_log_print(ANDROID_LOG_VERBOSE,LOG_TAG,__VA_ARGS__)
#define LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG,__VA_ARGS__)
#define LOGW(...)  __android_log_print(ANDROID_LOG_WARN,LOG_TAG,__VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)


#define		G711_A_LAW	(0)
#define		G711_MU_LAW	(1)
	
#define         SIGN_BIT        (0x80)      /* Sign bit for a A-law byte. */
#define         QUANT_MASK      (0xf)       /* Quantization field mask. */
#define         NSEGS           (8)         /* Number of A-law segments. */
#define         SEG_SHIFT       (4)         /* Left shift for segment number. */
#define         SEG_MASK        (0x70)      /* Segment field mask. */
#define         BIAS            (0x84)      /* Bias for linear code. */
#define			CLIP            8159
	
#define			DATA_LEN		(16)
	


class CmdControlListener : virtual public RefBase {
public:
	virtual void notify(int what, int ext1, char* ext2) = 0;
	virtual ~CmdControlListener(){}
private:
};


class PcmProcess{
public:
	PcmProcess(){};
	~PcmProcess(){}

	static short search(int val, short *table, int size);
	static unsigned char linear2alaw(int pcm_val);
	static int alaw2linear(unsigned char a_val);

	static unsigned char linear2ulaw(short pcm_val);
	static short ulaw2linear(unsigned char u_val);

	static unsigned char alaw2ulaw(unsigned char aval);
	static unsigned char ulaw2alaw(unsigned char uval);

	static int g711_encode(char *a_psrc, char *a_pdst, int in_data_len, unsigned char type);
	static int g711_decode(char *a_psrc, char *a_pdst, int in_data_len, unsigned char type);
};
#ifdef __cplusplus
}
#endif

}  // namespace cmdcontrol



#endif /* UTILS_H_ */
