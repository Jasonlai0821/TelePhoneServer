package com.xinshiyun.telephoneserver.widget;

import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.util.Log;

import java.lang.ref.WeakReference;

public class LcdDispJNI {
    private static String TAG = LcdDispJNI.class.getSimpleName();

    private EventHandler mEventHandler;
    private Context mContext;
    private CmdControlCallback mCmdControlCallback = null;

    private long mNativeContext; // accessed by native methods

    private class EventHandler extends Handler {
        private LcdDispJNI mLcdDispJNI;

        public EventHandler(LcdDispJNI dc, Looper looper) {
            super(looper);
            mLcdDispJNI = dc;
        }

        @Override
        public void handleMessage(Message msg) {
            if (mLcdDispJNI.mNativeContext == 0) {
                Log.w(TAG, "Display Control went away with unhandled events");
                return;
            }

            Log.d(TAG, "msg.what = " + msg.what);

            switch (msg.what) {

            }
        }
    }

    static {
        System.loadLibrary("LcdDispJNI");
        native_init();
    }

    public LcdDispJNI(Context context) {
        Looper looper;
        if ((looper = Looper.myLooper()) != null) {
            mEventHandler = new EventHandler(this, looper);
        } else if ((looper = Looper.getMainLooper()) != null) {
            mEventHandler = new EventHandler(this, looper);
        } else {
            mEventHandler = null;
        }

        native_setup(new WeakReference<LcdDispJNI>(this));
        mContext =context;
    }

    /*
     * Called from native code when an interesting event happens. This method
     * just uses the EventHandler system to post the event back to the main app
     * thread. We use a weak reference to the original DisplayControl object so
     * that the native code is safe from the object disappearing from underneath
     * it. (This is the cookie passed to native_setup().)
     */
    @SuppressWarnings("rawtypes")
    public static void postEventFromNative(Object dc_ref, int what, int arg1,int arg2) {
        LcdDispJNI dc = (LcdDispJNI) ((WeakReference) dc_ref).get();
        if (dc == null) {
            //return 0;
        }
        if (dc.mEventHandler != null) {
            Message m = dc.mEventHandler.obtainMessage(what, arg1, arg2);
            dc.mEventHandler.sendMessage(m);
        }
        //return 0;
    }

    private static native final void native_init();

    private native final void native_setup(Object dc_this);

    private native void native_finalize();

    public int signal_type_2G=2;
    public int signal_type_3G=3;
    public int signal_type_4G=4;
    public int signal_type_5G=5;

    public int phone_role_P1=1;
    public int phone_role_P2=2;
    public int phone_role_Pa=3;
    public int phone_role_Pb=4;

    public int phone_on_hook=1;
    public int phone_off_hook=2;
    public int phone_ringing=3;

    public native int onDispWelcome();
    public native int onDispLoading();
    public native int onDispNoSignal();
    public native int onDispRssi(int type,int strength);
    public native int onDispPxStatus(int px, int status, int time);

    public void finalize() throws Throwable {
        // TODO Auto-generated method stub
        super.finalize();
        if (mNativeContext != 0) {
            native_finalize();
        }
        mNativeContext = 0L;
    }

    public void setCmdControlCallback(CmdControlCallback callback){
        mCmdControlCallback = callback;
    }

    public interface CmdControlCallback{
        void onCmdExcuteResult(int cmd_type, int value);
    }
}
