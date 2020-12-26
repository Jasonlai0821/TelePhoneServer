package com.xinshiyun.telephoneserver.linphone;

import org.linphone.core.AuthInfo;
import org.linphone.core.AuthMethod;
import org.linphone.core.Call;
import org.linphone.core.Core;
import org.linphone.core.ProxyConfig;
import org.linphone.core.RegistrationState;

public abstract class PhoneServiceCallback {
    /**
     * 注册状态
     */
    public void onRegistrationStateChanged(Core lc, ProxyConfig cfg, RegistrationState cstate, String message) {
    }

    /**
     * 注册状态
     */
    public void unRegistrationStateChanged(Core lc, AuthInfo authInfo, AuthMethod method) {
    }
    /**
     * 来电状态
     *
     * @param linphoneCall
     */
    public void incomingCall(Call linphoneCall) {
    }

    /**
     * 电话接通
     */
    public void callConnected() {
    }

    /**
     * 电话被挂断
     */
    public void callReleased() {
    }

    /**
     * 正在呼叫
     */
    public void OutgoingProgress(){
    }

}
