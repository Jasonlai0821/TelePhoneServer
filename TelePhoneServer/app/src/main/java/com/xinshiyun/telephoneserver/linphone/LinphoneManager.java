package com.xinshiyun.telephoneserver.linphone;

import android.content.Context;
import android.os.Environment;
import android.os.Handler;
import android.util.Log;
import com.xinshiyun.telephoneserver.R;
import org.linphone.core.Core;
import org.linphone.core.CoreListener;
import org.linphone.core.Factory;
import org.linphone.core.LogCollectionState;
import java.io.File;
import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;

/**
 * 初始化 linphone
 */
public class LinphoneManager {
    private static String TAG = LinphoneManager.class.getSimpleName();
    private Context mContext;
    private static LinphoneManager instance;
    private static boolean sExited;

    private String mLinphoneFactoryConfigFile = null;
    public String mLinphoneConfigFile = null;
    private String mLPConfigXsd = null;
    private String mLinphoneRootCaFile = null;
    private String mRingSoundFile = null;
    private String mRingBackSoundFile = null;
    private String mPauseSoundFile = null;
    private String mChatDatabaseFile = null;
    private String mUserCerts = null;

    private Core mCore;
    private CoreListener mCoreListener;

    private Timer mTimer;
    private Handler mHandler;

    public LinphoneManager(Context context) {
        mContext = context;
        sExited = false;

        String basePath = Environment.getExternalStorageDirectory().getAbsolutePath();
        mLPConfigXsd = basePath + "/lpconfig.xsd";
        mLinphoneFactoryConfigFile = basePath + "/linphonerc";
        mLinphoneConfigFile = basePath + "/linphonerc";

        mLinphoneRootCaFile = basePath + "/rootca.pem";
        mRingSoundFile = basePath + "/dont_wait_too_long.mkv"; //dont_wait_too_long.mkv   oldphone_mono.wav
        mRingBackSoundFile = basePath + "/ringback.wav";
        mPauseSoundFile = basePath + "/toy_mono.wav";
        mChatDatabaseFile = basePath + "/linphone-history.db";
        mUserCerts = basePath + "/user-certs";

        Factory.instance().setLogCollectionPath(basePath);
        Factory.instance().enableLogCollection(LogCollectionState.Enabled); //日志开关
        Factory.instance().setDebugMode(true, "Linphone");

        mHandler = new Handler();
    }

    public synchronized static final LinphoneManager createAndStart(Context context, CoreListener coreListener) {
        Log.e(TAG, "createAndStart()");
        if (instance != null) {
            throw new RuntimeException("Linphone Manager is already initialized");
        }

        instance = new LinphoneManager(context);
        instance.startLibLinphone(context, coreListener);
        return instance;
    }

    private synchronized void startLibLinphone(Context context, CoreListener coreListener) {
        Log.e(TAG, "startLibLinphone()");
        try {
            mCoreListener = coreListener;
            copyAssetsFromPackage();

            // Create the Core and add our listener
            mCore = Factory.instance().createCore(mLinphoneConfigFile, mLinphoneFactoryConfigFile, context);
            Log.i(TAG,mCore.getRingback()+"");
            mCore.addListener(coreListener);

            initLibLinphone();

            // Core must be started after being created and configured
            mCore.start();
            // We also MUST call the iterate() method of the Core on a regular basis
            TimerTask lTask = new TimerTask() {
                @Override
                public void run() {
                    mHandler.post(new Runnable() {
                        @Override
                        public void run() {
                            if (mCore != null) {
                                mCore.iterate();
                            }
                        }
                    });
                }
            };
            mTimer = new Timer("Linphone scheduler");
            mTimer.schedule(lTask, 0, 20);
        } catch (IOException e) {
            e.printStackTrace();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void copyAssetsFromPackage() throws IOException {
        Log.e(TAG, "copyAssetsFromPackage()");
        LinphoneUtils.copyIfNotExist(mContext, R.raw.default_assistant_create, mLinphoneConfigFile);
        LinphoneUtils.copyIfNotExist(mContext, R.raw.linphone_assistant_create, new File(mLinphoneFactoryConfigFile).getName());
    }

    private void initLibLinphone() {
        Log.e(TAG, "initLibLinphone()");
        File f = new File(mUserCerts);
        if (!f.exists()) {
            if (!f.mkdir()) {
                Log.e("TAG",mUserCerts + " can't be created.");
            }
        }
        mCore.setUserCertificatesPath(mUserCerts);

    }


    public static synchronized Core getCoreIfManagerNotDestroyOrNull() {
        Log.e(TAG, "getCoreIfManagerNotDestroyOrNull()");
        if (sExited || instance == null) {
            Log.e("TAG","Trying to get linphone core while LinphoneManager already destroyed or not created");
            return null;
        }
        return getCore();
    }

    public static synchronized final Core getCore() {
        Log.e(TAG, "getCore()");
        return getInstance().mCore;
    }


    public static final boolean isInstanceiated() {
        return instance != null;
    }

    public static synchronized final LinphoneManager getInstance() {
        if (instance != null) {
            return instance;
        }
        if (sExited) {
            throw new RuntimeException("Linphone Manager was already destroyed. " + "Better use getLcIfManagerNotDestroyed and check returned value");
        }
        throw new RuntimeException("Linphone Manager should be created before accessed");
    }

    public static void destroy() {
        Log.e(TAG, "destroy()");
        if (instance == null) {
            return;
        }
        sExited = true;
        instance.doDestroy();
    }

    public void doDestroy() {
        try {
            mCore.removeListener(mCoreListener);
            mTimer.cancel();
            mCore.stop();
        } catch (RuntimeException e) {
            e.printStackTrace();
        } finally {
            mCore = null;
            instance = null;
        }
    }

    public void registerUserAuth()
    {
        Log.d(TAG,"registerUserAuth()");
        PhoneVoiceUtils.getInstance().registerUserAuth("8001", "xZQHsR", "153.35.93.16:6089");
    }

    public void startSingleCall(String callnumber)
    {
        Log.d(TAG,"startSingleCall() callnumber ="+callnumber);
        PhoneVoiceUtils.getInstance().startSingleCallingTo(callnumber);
    }

    public void stopSingleCall()
    {
        Log.d(TAG,"stopSingleCall()");
        PhoneVoiceUtils.getInstance().hangUp();
    }

}