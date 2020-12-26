package com.xinshiyun.telephoneserver.widget;

import android.content.Context;
import android.content.Intent;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.util.Log;

import java.lang.ref.WeakReference;

public class CmdControlJNI {
    private static String TAG = CmdControlJNI.class.getSimpleName();

    private EventHandler mEventHandler;
    private Context mContext;
    private CmdControlCallback mCmdControlCallback = null;

    private long mNativeContext; // accessed by native methods
    public static final int 		REM_ACTIVE_RING			=0x00;
    public static final int 		REM_ACTIVE_CALLID		=0x01;
    public static final int 		REM_ACTIVE_BUSY		    =0x02;
    public static final int 		REM_ACTIVE_AUDIO	    =0x03;
    public static final int 		TERMINAL_ACTIVE_RING	=0x04;
    public static final int 		TERMINAL_ACTIVE_CALLID	=0x05;
    public static final int 		TERMINAL_ACTIVE_PARTY_NUM=0x06;
    public static final int 		TERMINAL_ACTIVE_AUDIO	=0x07;
    public static final int 		TERMINAL_BUSY	        =0x08;
    public static final int 		REM_ACTIVE_CANCEL	    =0x09;

    private class EventHandler extends Handler {
        private CmdControlJNI mCmdControlJNI;

        public EventHandler(CmdControlJNI dc, Looper looper) {
            super(looper);
            mCmdControlJNI = dc;
        }

        @Override
        public void handleMessage(Message msg) {
            if (mCmdControlJNI.mNativeContext == 0) {
                Log.w(TAG, "TelePhone Control went away with unhandled events");
                return;
            }

            Log.w(TAG, "TelePhone Control listener msg what= "+ msg.what);
            switch (msg.what) {
                case REM_ACTIVE_RING:
                    if(mCmdControlCallback != null){
                        mCmdControlCallback.onCmdExcuteResult(REM_ACTIVE_RING,msg.arg1,null);
                    }
                    break;
                case REM_ACTIVE_CALLID:
                    String result = (String) msg.obj;
                    if(mCmdControlCallback != null && result != null){
                        Log.w(TAG, "TelePhone Control listener callid= "+ result);
                        mCmdControlCallback.onCmdExcuteResult(msg.what,msg.arg1,result);
                    }
                    break;
                case REM_ACTIVE_BUSY:
                    if(mCmdControlCallback != null ){
                        mCmdControlCallback.onCmdExcuteResult(msg.what,msg.arg1,null);
                    }
                    break;
                case REM_ACTIVE_AUDIO:
                    break;
                case REM_ACTIVE_CANCEL:
                    if(mCmdControlCallback != null ){
                        mCmdControlCallback.onCmdExcuteResult(msg.what,msg.arg1,null);
                    }
                    break;
                default:
                    break;
            }
        }
    }

    static {
        System.loadLibrary("CmdControlJNI");
        native_init();
    }

    public CmdControlJNI(Context context) {
        Looper looper;
        if ((looper = Looper.myLooper()) != null) {
            mEventHandler = new EventHandler(this, looper);
        } else if ((looper = Looper.getMainLooper()) != null) {
            mEventHandler = new EventHandler(this, looper);
        } else {
            mEventHandler = null;
        }

        native_setup(new WeakReference<CmdControlJNI>(this));
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
    public static void postEventFromNative(Object dc_ref, int what, int arg1, String arg2) {
        CmdControlJNI dc = (CmdControlJNI) ((WeakReference) dc_ref).get();
        if (dc == null) {
            return;
        }
        if (dc.mEventHandler != null) {
            Message message = Message.obtain();
            message.what = what;
            message.arg1 = arg1;
            message.obj = arg2;
            //Message m = dc.mEventHandler.obtainMessage(what, arg1, arg2);
            dc.mEventHandler.sendMessage(message);
        }
        //return 0;
    }

    public void rebootSystem()
    {
        Intent intent = new Intent(Intent.ACTION_REBOOT);
        intent.putExtra("nowait", 1);
        intent.putExtra("interval", 1);
        intent.putExtra("window", 0);
        mContext.sendBroadcast(intent);
    }

    //串口指令的格式是Lead code + Length +cmd + data +crc
    //如下的cmd缺省crc，crc值由底层自动计算
    public String getValue(int value)
    {
        String val = null;
        val = Integer.toHexString(value);
        if(val.length() == 1){
            val = "0"+val;
        }
        val = val.toUpperCase();
        return val;
    }

    public void onTerminalActiveRing()
    {
        String cmd = "5A"+"05"+"04";
        Log.d(TAG,"onTerminalActiveRing() cmd:"+cmd);
        onExecuteCmd(cmd);
    }

    public int onTerminalActiveCallId(String callid)
    {
        String cmd = "5A"+"11"+"05";//+时间戳和CallId
        Log.d(TAG,"onTerminalActiveCallId() cmd:"+cmd);
        return onExecuteCmd(cmd);
    }

    public void onTerminalActivePartynum(String num)
    {
        String cmd = "5A"+"07"+"06"+num;

        Log.d(TAG,"onTerminalActivePartynum() cmd:"+cmd);
        onExecuteCmd(cmd);
    }

    public int onTerminalOffHook()
    {
        String cmd = "5A"+"05"+"08";
        Log.d(TAG,"onTerminalOffHook() cmd:"+cmd);
        return onExecuteCmd(cmd);
    }

    public void onTerminalOnHook()
    {
        String cmd = "5A"+"05"+"08";
        Log.d(TAG,"onTerminalOnHook() cmd:"+cmd);
        onExecuteCmd(cmd);
    }

    public void onStartCommunication()
    {
        String cmd = "5A"+"05"+"FE";
        Log.d(TAG,"onStartCommunication() cmd:"+cmd);
        onExecuteCmd(cmd);
    }

    public void onStopCommunication()
    {
        String cmd = "5A"+"05"+"FF";
        Log.d(TAG,"onStartCommunication() cmd:"+cmd);
        onExecuteCmd(cmd);
    }

    public void onPlayPcmData(byte[] pcmData,int len)
    {
        Log.d(TAG,"onPlayPcmData() len ="+len);
        onProcessPCMData(pcmData,len);
    }

    private static native final void native_init();

    private native final void native_setup(Object dc_this);

    private native void native_finalize();

    public native int onExecuteCmd(String cmd);

    public native int onProcessPCMData(byte[] audioData,int len);

    @Override
    protected void finalize() throws Throwable {
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
        void onCmdExcuteResult(int cmd_type, int value, String result);
    }
}
