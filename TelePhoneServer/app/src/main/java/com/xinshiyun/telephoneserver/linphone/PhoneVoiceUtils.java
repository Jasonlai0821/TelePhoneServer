package com.xinshiyun.telephoneserver.linphone;

import android.os.Environment;
import android.util.Log;

import org.linphone.core.AccountCreator;
import org.linphone.core.Address;
import org.linphone.core.AuthInfo;
import org.linphone.core.Call;
import org.linphone.core.CallParams;
import org.linphone.core.Core;
import org.linphone.core.ProxyConfig;
import org.linphone.core.TransportType;

public class PhoneVoiceUtils {
    private static String TAG = PhoneVoiceUtils.class.getSimpleName();
    private static volatile PhoneVoiceUtils sPhoneVoiceUtils;
    private Core mLinphoneCore = null;
    private PhoneServiceCallback sPhoneServiceCallback;


    public void addCallback(PhoneServiceCallback phoneServiceCallback) {
        sPhoneServiceCallback = phoneServiceCallback;
    }


    public static PhoneVoiceUtils getInstance() {
        Log.d(TAG, "getInstance()");
        if (sPhoneVoiceUtils == null) {
            synchronized (PhoneVoiceUtils.class) {
                if (sPhoneVoiceUtils == null) {
                    sPhoneVoiceUtils = new PhoneVoiceUtils();
                }
            }
        }
        return sPhoneVoiceUtils;
    }

    private PhoneVoiceUtils() {
        Log.d(TAG, "PhoneVoiceUtils()");
        mLinphoneCore = LinphoneManager.getCore();
        mLinphoneCore.enableEchoCancellation(true);
        mLinphoneCore.enableEchoLimiter(true);

    }

    /**
     * 注册到服务器
     *
     * @param name     账号名
     * @param password 密码
     * @param host     IP地址：端口号
     */
    public void registerUserAuth(String name, String password, String host) {
        Log.d(TAG, "registerUserAuth()");
        registerUserAuth(name, password, host, TransportType.Udp);
    }

    /**
     * 注册到服务器
     *
     * @param name     账号名
     * @param password 密码
     * @param host     IP地址：端口号
     * @param type     TransportType.Udp TransportType.Tcp TransportType.Tls
     */
    public void registerUserAuth(String name, String password, String host, TransportType type) {
        Log.d(TAG, "registerUserAuth()");
        //    String identify = "sip:" + name + "@" + host;
        AccountCreator mAccountCreator = mLinphoneCore.createAccountCreator(null);

        mAccountCreator.setUsername(name);
        mAccountCreator.setDomain(host);
        mAccountCreator.setPassword(password);
        mAccountCreator.setTransport(type);

        ProxyConfig cfg = mAccountCreator.createProxyConfig();
        // Make sure the newly created one is the default
        mLinphoneCore.setDefaultProxyConfig(cfg);
    }

    //取消注册
    public void unRegisterUserAuth() {
        Log.d(TAG, "unRegisterUserAuth()");
        mLinphoneCore.clearAllAuthInfo();
        logRegisterUserAuth();

    }

    //注册信息
    public void logRegisterUserAuth() {
        Log.e(TAG, "---- 注册信息" + mLinphoneCore);

    }

    /**
     * 是否已经注册了
     *
     * @return
     */
    public boolean isRegistered() {
        Log.d(TAG, "isRegistered()");
        AuthInfo[] authInfos = mLinphoneCore.getAuthInfoList();
        if (authInfos.length > 0) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * 拨打电话
     *
     * @param phone 手机号
     * @return
     */
    public Call startSingleCallingTo(String phone) {
        Log.d(TAG, "startSingleCallingTo()");
        Call call = null;
        try {
            Address addressToCall = mLinphoneCore.interpretUrl(phone);

            CallParams params = mLinphoneCore.createCallParams(null);

            params.enableVideo(false); //不可视频
            String str = params.getRecordFile();
            params.setRecordFile(Environment.getExternalStorageDirectory().getAbsolutePath()
                    + "/recorderdemo11/" + System.currentTimeMillis() + ".mp4");
            if (addressToCall != null) {
                call = mLinphoneCore.inviteAddressWithParams(addressToCall, params);
            }
        } catch (Exception e) {
            e.printStackTrace();

        }
        return call;
    }

    /**
     * 挂断电话
     */
    public void hangUp() {
        Log.d(TAG, "hangUp()");
        if (mLinphoneCore == null) {
            mLinphoneCore = LinphoneManager.getCore();
        }

        Call currentCall = mLinphoneCore.getCurrentCall();
        if (currentCall != null) {
            mLinphoneCore.terminateCall(currentCall);
        } else if (mLinphoneCore.isInConference()) {
            mLinphoneCore.terminateConference();
        } else {
            mLinphoneCore.terminateAllCalls();
        }
        unRegisterUserAuth();
    }

    /**
     * 是否静音
     *
     * @param isMicMuted
     */
    public void toggleMicro(boolean isMicMuted) {
        Log.d(TAG, "toggleMicro()");
        if (mLinphoneCore == null) {
            mLinphoneCore = LinphoneManager.getCore();
        }
        mLinphoneCore.enableMic(isMicMuted);
    }

    /**
     * 接听来电
     *
     * @param
     */
    public void receiveCall(Call call) {
        Log.d(TAG, "receiveCall()");
        if (mLinphoneCore == null) {
            mLinphoneCore = LinphoneManager.getCore();
        }
        CallParams params = mLinphoneCore.createCallParams(call);
        params.enableVideo(false);
        if (null != call) {
            call.acceptWithParams(params);
        }
    }

}