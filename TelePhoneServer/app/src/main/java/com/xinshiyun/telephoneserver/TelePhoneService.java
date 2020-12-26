package com.xinshiyun.telephoneserver;

import android.app.Service;
import android.content.Intent;
import android.os.IBinder;
import android.os.RemoteCallbackList;
import android.os.RemoteException;
import android.util.Log;

import androidx.annotation.Nullable;

public class TelePhoneService extends Service {
    private static String TAG = TelePhoneService.class.getSimpleName();
    private static TelePhoneService sInstance;
    public final RemoteCallbackList<ITelePhoneCallback> mTelePhoneCallbacks = new RemoteCallbackList<ITelePhoneCallback>();
    private TelePhone mBinder;
    private static TelePhoneServiceImp mTelePhoneServiceImp = null;
    private TelePhoneServiceListener mTelePhoneServiceListener = null;

    @Override
    public void onCreate() {
        super.onCreate();
        mBinder = new TelePhone();

        mTelePhoneServiceImp = TelePhoneServiceImp.getInstance(this);
        mTelePhoneServiceListener = new TelePhoneServiceListener();
        mTelePhoneServiceImp.setTelePhoneServiceCallback(mTelePhoneServiceListener);
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        mTelePhoneServiceImp.destroy();
        mTelePhoneServiceImp = null;
    }

    @Nullable
    @Override
    public IBinder onBind(Intent intent) {
        return mBinder;
    }

    public class TelePhone extends ITelePhoneBinder.Stub{

        @Override
        public void registerTelePhoneCallback(ITelePhoneCallback callback) throws RemoteException {
            Log.d(TAG, "registerTelePhoneCallback()");
            if (callback != null) {
                mTelePhoneCallbacks.register(callback);
            }
        }

        @Override
        public void unregisterTelePhoneCallback(ITelePhoneCallback callback) throws RemoteException {
            Log.d(TAG, "unregisterTelePhoneCallback()");
            if (callback != null) {
                mTelePhoneCallbacks.unregister(callback);
            }
        }

        @Override
        public int startCall(String callnumber) throws RemoteException {
            Log.d(TAG, "startCall()");
            return onStartCall(callnumber);
        }

        @Override
        public int stopCall(String callnumber) throws RemoteException {
            Log.d(TAG, "stopCall()");
            return onStopCall(callnumber);
        }

        @Override
        public int getCallState() throws RemoteException {
            return onGetCallState();
        }
    }

    public int onStartCall(String callnumber)
    {
        Log.d(TAG, "onStartCall()");

        if(mTelePhoneServiceImp != null){
            return mTelePhoneServiceImp.onStartCall(callnumber);
        }
        return -1;
    }

    public int onStopCall(String callnumber)
    {
        Log.d(TAG, "onStartCall()");

        if(mTelePhoneServiceImp != null){
            return mTelePhoneServiceImp.onStopCall(callnumber);
        }
        return -1;
    }

    public int onGetCallState()
    {
        Log.d(TAG, "onGetCallState()");

        if(mTelePhoneServiceImp != null){
            return mTelePhoneServiceImp.onGetCallState();
        }
        return -1;
    }

    public class TelePhoneServiceListener implements TelePhoneServiceImp.TelePhoneServiceCallback{

        @Override
        public void onCallState(int state) {
            Log.d(TAG, "onCallState()");
            if (mTelePhoneCallbacks == null) {
                return;
            }
            try {
                final int N = mTelePhoneCallbacks.beginBroadcast();
                for (int i = 0; i < N; i++) {
                    mTelePhoneCallbacks.getBroadcastItem(i).onCallState(state);
                }
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
                mTelePhoneCallbacks.finishBroadcast();
            }
        }

        @Override
        public void onSuccess(int state) {
            Log.d(TAG, "onSuccess()");
            if (mTelePhoneCallbacks == null) {
                return;
            }
            try {
                final int N = mTelePhoneCallbacks.beginBroadcast();
                for (int i = 0; i < N; i++) {
                    mTelePhoneCallbacks.getBroadcastItem(i).OnSuccess(state);
                }
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
                mTelePhoneCallbacks.finishBroadcast();
            }
        }

        @Override
        public void onFailure(int state) {
            Log.d(TAG, "onFailure()");
            if (mTelePhoneCallbacks == null) {
                return;
            }
            try {
                final int N = mTelePhoneCallbacks.beginBroadcast();
                for (int i = 0; i < N; i++) {
                    mTelePhoneCallbacks.getBroadcastItem(i).onFailure(state);
                }
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
                mTelePhoneCallbacks.finishBroadcast();
            }
        }
    }
}
