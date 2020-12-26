// ITelePhoneBinder.aidl
package com.xinshiyun.telephoneserver;

import com.xinshiyun.telephoneserver.ITelePhoneCallback;
// Declare any non-default types here with import statements

interface ITelePhoneBinder {
   void registerTelePhoneCallback(ITelePhoneCallback callback);
   void unregisterTelePhoneCallback(ITelePhoneCallback callback);

   int startCall(String callnumber);
   int stopCall(String callnumber);
   int getCallState();
}
