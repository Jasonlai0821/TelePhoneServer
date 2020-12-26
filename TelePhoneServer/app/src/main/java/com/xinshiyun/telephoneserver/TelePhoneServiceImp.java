package com.xinshiyun.telephoneserver;

import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.Message;
import android.util.Log;
import android.widget.Toast;

import com.xinshiyun.telephoneserver.linphone.LinphoneManager;
import com.xinshiyun.telephoneserver.widget.CmdControlJNI;
import com.xinshiyun.telephoneserver.widget.LcdDispJNI;

import org.linphone.core.Call;
import org.linphone.core.CallStats;
import org.linphone.core.Core;
import org.linphone.core.CoreListenerStub;
import org.linphone.core.ProxyConfig;
import org.linphone.core.RegistrationState;

public class TelePhoneServiceImp {
    private final static String TAG = TelePhoneServiceImp.class.getSimpleName();
    private static  TelePhoneServiceImp mTelePhoneServiceImp = null;

    private CmdControlJNI mCmdControlJNI = null;
    private LcdDispJNI mLcdDispJNI = null;
    private TelePhoneCtrlCallback mTelePhoneCtrlCallback = null;

    private LinPhoneCoreListener mLinPhoneCoreListener;
    private static String CurrentCallNumber = "15280225861";
    private static Call.State CurrentCallState = Call.State.Idle;
    private TelePhoneServiceCallback mTelePhoneListener = null;

    public Context mContext;

    public static final class MSG{
        public static final int MSG_ON_DSP_REM_RING = 0;
        public static final int MSG_ON_DSP_REM_ONHOOK = MSG_ON_DSP_REM_RING+1;
        public static final int MSG_ON_DSP_REM_CALLID = MSG_ON_DSP_REM_ONHOOK+1;
        public static final int MSG_ON_TERMINAL_RING = MSG_ON_DSP_REM_CALLID+1;
        public static final int MSG_ON_TERMINAL_ONHOOK = MSG_ON_TERMINAL_RING+1;
        public static final int MSG_ON_STARTCOMMUNICTION = MSG_ON_TERMINAL_ONHOOK+1;
        public static final int MSG_ON_STOPCOMMUNICTION = MSG_ON_STARTCOMMUNICTION+1;
        public static final int MSG_ON_UNKNOWN = MSG_ON_TERMINAL_ONHOOK+1;
    }

    public TelePhoneServiceImp(Context context)
    {
        this.mContext = context;
        //初始化Core
        initResource();
        initWidgetTools();
        initLinPhone();
    }

    public synchronized static TelePhoneServiceImp getInstance(Context context)
    {
        Log.d(TAG,"TelePhoneServiceImp getInstance()");
        if (mTelePhoneServiceImp == null) {
            mTelePhoneServiceImp = new TelePhoneServiceImp(context);
        }

        return mTelePhoneServiceImp;
    }

    public void initWidgetTools()
    {
        mCmdControlJNI = new CmdControlJNI(mContext);
        mLcdDispJNI = new LcdDispJNI(mContext);
        mTelePhoneCtrlCallback = new TelePhoneCtrlCallback();
        mCmdControlJNI.setCmdControlCallback(mTelePhoneCtrlCallback);
    }

    public void initResource()
    {

    }

    public void deInitWidgetTools()
    {
        mCmdControlJNI = null;
        mLcdDispJNI = null;
    }

    public void initLinPhone()
    {
        mLinPhoneCoreListener = new LinPhoneCoreListener();
        LinphoneManager.createAndStart(mContext, mLinPhoneCoreListener);
    }

    public void destroy()
    {
        deInitWidgetTools();
        LinphoneManager.destroy();
    }

    public void setTelePhoneServiceCallback(TelePhoneServiceCallback callback)
    {
        Log.d(TAG, "setTelePhoneServiceCallback()");
        mTelePhoneListener = callback;
    }

    public interface TelePhoneServiceCallback{
        void onCallState(int state);

        void onSuccess(int state);

        void onFailure(int state);
    }

    private class TelePhoneCtrlCallback implements CmdControlJNI.CmdControlCallback{

        @Override
        public void onCmdExcuteResult(int cmd_type, int value, String result) {
            Log.d(TAG,"onCmdExcuteResult()cmd_type:" +cmd_type);
            switch (cmd_type) {
                case CmdControlJNI.REM_ACTIVE_RING:
                    mHandler.sendEmptyMessage(MSG.MSG_ON_DSP_REM_RING);
                    break;
                case CmdControlJNI.REM_ACTIVE_BUSY:
                    mHandler.sendEmptyMessage(MSG.MSG_ON_DSP_REM_ONHOOK);
                    break;
                case CmdControlJNI.REM_ACTIVE_CALLID:
                    Log.d(TAG,"onCmdExcuteResult() callid number:" +result);
                    break;
                case CmdControlJNI.REM_ACTIVE_AUDIO:
                    break;
                case CmdControlJNI.REM_ACTIVE_CANCEL:
                    Log.d(TAG,"REM cancel !!");
                    //Toast.makeText(mContext, "对方已取消！", Toast.LENGTH_LONG).show();
                    mHandler.sendEmptyMessage(MSG.MSG_ON_DSP_REM_ONHOOK);
                    break;
                default:
                    break;
            }
        }
    }

    private Handler mHandler = new Handler(Looper.getMainLooper()){
        @Override
        public void handleMessage(Message msg){
            Log.d(TAG,"handleMessage() msg what:" + msg.what);
            switch (msg.what){
                case MSG.MSG_ON_DSP_REM_RING:
                    Log.d(TAG,"handleMessage() MSG_ON_DSP_REM_RING");
                    if(mLcdDispJNI != null  && CurrentCallState == Call.State.Idle){
                        mLcdDispJNI.onDispPxStatus(mLcdDispJNI.phone_role_Pa, mLcdDispJNI.phone_ringing, 8);//对方振铃
                    }
                    onStartCall(CurrentCallNumber);
                    break;
                case MSG.MSG_ON_DSP_REM_ONHOOK:
                    Log.d(TAG,"handleMessage() MSG_ON_DSP_REM_ONHOOK");
                    if(mLcdDispJNI != null && CurrentCallState != Call.State.Idle){
                        mLcdDispJNI.onDispPxStatus(mLcdDispJNI.phone_role_Pa, mLcdDispJNI.phone_on_hook, 8);//挂机
                    }
                    onStopCall(CurrentCallNumber);
                    break;
                case MSG.MSG_ON_DSP_REM_CALLID:
                    Log.d(TAG,"handleMessage() MSG_ON_DSP_REM_CALLID");
                    break;
                case MSG.MSG_ON_TERMINAL_RING:
                    Log.d(TAG,"handleMessage() MSG_ON_TERMINAL_RING");
                    if(mCmdControlJNI !=null){
                        mCmdControlJNI.onTerminalActiveRing();
                    }
                    break;
                case MSG.MSG_ON_TERMINAL_ONHOOK:
                    Log.d(TAG,"handleMessage() MSG_ON_TERMINAL_ONHOOK");
                    if(mCmdControlJNI !=null){
                        //mCmdControlJNI.onStopCommunication();
                        //mCmdControlJNI.onStartCommunication();
                        mCmdControlJNI.onTerminalOnHook();
                    }
                    break;
                case MSG.MSG_ON_STARTCOMMUNICTION:
                    Log.d(TAG,"handleMessage() MSG_ON_STARTCOMMUNICTION");
                    if(mCmdControlJNI !=null){
                        mCmdControlJNI.onStartCommunication();
                    }
                    break;
            }
        }
    };

    public int onStartCall(String callnumber)
    {
        if(callnumber != null && CurrentCallState == Call.State.Idle){
            Log.d(TAG,"onStartCall() callnumber:" + callnumber);
            CurrentCallNumber = callnumber;
            LinphoneManager.getInstance().registerUserAuth();
        }else{
            Log.e(TAG,"onStartCall() callnumber is null or Current is calling");
        }

        return 0;
    }

    public int onStopCall(String callnumber)
    {
        Log.d(TAG,"onStopCall()");
        //if(CurrentCallState != Call.State.Idle){
            LinphoneManager.getInstance().stopSingleCall();
            //CurrentCallNumber = null;
            mHandler.sendEmptyMessage(MSG.MSG_ON_TERMINAL_ONHOOK);
            CurrentCallState = Call.State.Idle;
        //}
        return 0;
    }

    public int onGetCallState()
    {
        int ret = CurrentCallState.toInt();
        return ret;
    }

    private class LinPhoneCoreListener extends CoreListenerStub {
        /**
         *  通话状态
         * @param lc
         * @param call
         * @param cstate
         * @param message
         */
        @Override
        public void onCallStateChanged(Core lc, Call call, Call.State cstate, String message){
            Log.d(TAG,"onCallStateChanged() State =" + cstate +" Message:" + message);
            Call.State mTempState = CurrentCallState;
            CurrentCallState = cstate;
            if (cstate == Call.State.OutgoingInit || cstate == Call.State.OutgoingProgress){//正在呼叫
                if(mTelePhoneListener != null) {
                    mTelePhoneListener.onSuccess(CurrentCallState.toInt());
                }
            }else if(cstate == Call.State.OutgoingEarlyMedia || cstate == Call.State.OutgoingRinging){
                if(mTempState == Call.State.OutgoingRinging){
                    if(mCmdControlJNI !=null){
                        mCmdControlJNI.onStopCommunication();
                    }
                }
                if(mCmdControlJNI !=null){
                    mCmdControlJNI.onStartCommunication();
                }
            }else if (cstate == Call.State.Connected || cstate == Call.State.StreamsRunning) { //接通
                if(mTempState != Call.State.Connected && (CurrentCallState == Call.State.Connected || CurrentCallState == Call.State.StreamsRunning)){
                    mHandler.sendEmptyMessage(MSG.MSG_ON_TERMINAL_RING);
                }
                if(mTelePhoneListener != null){
                    mTelePhoneListener.onSuccess(CurrentCallState.toInt());
                }
            }else if (cstate == Call.State.End || (cstate == Call.State.Released)) { //挂断，未接
                boolean callrelease_flg = false;
                if(cstate == Call.State.Released){
                    callrelease_flg = true;
                    onStopCall(null);
                    CurrentCallState = Call.State.Idle;
                    if(mTelePhoneListener != null){
                        mTelePhoneListener.onSuccess(CurrentCallState.toInt());
                    }
                }else if(cstate == Call.State.End){
                    callrelease_flg = true;
                }

                if(callrelease_flg == true){
                    Log.d(TAG,"onCallStateChanged() END or RELEASE send MSG_ON_TERMINAL_ONHOOK");
                    mHandler.sendEmptyMessage(MSG.MSG_ON_TERMINAL_ONHOOK);
                }
                Log.d(TAG,"onCallStateChanged() End Error info =" +call.getErrorInfo().getReason());
            }else if(cstate == Call.State.Error){
                Log.d(TAG,"onCallStateChanged() Error send MSG_ON_TERMINAL_ONHOOK");
                mHandler.sendEmptyMessage(MSG.MSG_ON_TERMINAL_ONHOOK);
                if(mTelePhoneListener != null){
                    mTelePhoneListener.onFailure(CurrentCallState.toInt());
                }
                Log.d(TAG,"onCallStateChanged() State Error info =" +call.getErrorInfo().getReason());
            }
            if(mTelePhoneListener != null){
                mTelePhoneListener.onCallState(CurrentCallState.toInt());
            }
        }

        /**
         * 注册状态
         * @param lc
         * @param cfg
         * @param cstate
         * @param message
         */
        @Override
        public void onRegistrationStateChanged(Core lc, ProxyConfig cfg, RegistrationState cstate, String message) {
            Log.d(TAG,"onRegistrationStateChanged() State =" + cstate +" Message:" + message);
            if(cstate == RegistrationState.Ok && "Registration successful".equals(message)){
                if(CurrentCallNumber != null && CurrentCallState == Call.State.Idle){
                    LinphoneManager.getInstance().startSingleCall(CurrentCallNumber);
                }else{
                    Log.d(TAG,"onRegistrationStateChanged() CurrentCallNumber is null");
                }
                CurrentCallState = Call.State.OutgoingInit;
                if(mTelePhoneListener != null){
                    mTelePhoneListener.onSuccess(CurrentCallState.toInt());
                }
            }else if(cstate == RegistrationState.Failed){
                Log.d(TAG,"onRegistrationStateChanged() Failed");
                if(mTelePhoneListener != null){
                    mTelePhoneListener.onFailure(CurrentCallState.toInt());
                }
                //mHandler.sendEmptyMessage(MSG.MSG_ON_TERMINAL_ONHOOK);
            }
            if(mTelePhoneListener != null){
                mTelePhoneListener.onCallState(CurrentCallState.toInt());
            }
        }

        public void onCallStatsUpdated(Core lc, Call call, CallStats stats){
            Log.e(TAG, "onCallStatsUpdated IceState state ="+stats.getIceState() +" UpnpState state ="+stats.getUpnpState());
        }
    }
}
