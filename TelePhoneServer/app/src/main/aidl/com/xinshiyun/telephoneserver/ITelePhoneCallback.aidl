// ITelePhoneCallback.aidl
package com.xinshiyun.telephoneserver;

// Declare any non-default types here with import statements

interface ITelePhoneCallback {
    void onCallState(int state);
    void OnSuccess(int state);
    void onFailure(int state);
}
