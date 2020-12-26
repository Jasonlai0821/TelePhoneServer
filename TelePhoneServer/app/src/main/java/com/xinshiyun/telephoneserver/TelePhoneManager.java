package com.xinshiyun.telephoneserver;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.pm.PackageManager;
import android.content.pm.ResolveInfo;
import android.os.IBinder;
import android.os.RemoteException;
import android.util.Log;

import com.xinshiyun.telephoneserver.misc.SysProperties;

import java.util.List;

public class TelePhoneManager {
    private static String TAG = TelePhoneManager.class.getSimpleName();
    private Context mContext;
    private static TelePhoneManager mTelePhoneManager = null;
    private ITelePhoneBinder mTelePhoneBinder = null;
    private ConnectionCallback mConnectionCallback;
    private TelePhoneListener mTelePhoneCallback = null;

    private static String INTENT_TELEPHONE_SERVICE = "com.xinshiyun.telephoneserver.TelePhoneService";
    private static String INTENT_TELEPHONE_CATEGORY = "com.xinshiyun.telephoneserver.default";

    public TelePhoneManager(Context context){
        Log.d(TAG,"TelePhoneManager()");
        this.mContext = context;
        mTelePhoneBinder = null;
        initTelePhone();
    }

    public synchronized static TelePhoneManager getInstance(Context context) {
        Log.d(TAG,"TelePhoneManager getInstance()");
        if (mTelePhoneManager == null) {
            mTelePhoneManager = new TelePhoneManager(context);
        }

        return mTelePhoneManager;
    }

    void initTelePhone()
    {
        Log.d(TAG,"initTelePhone()");
        SysProperties.setAudioVirtualState(mContext,true);
        mTelePhoneCallback = new TelePhoneListener();
        mConnectionCallback = new ConnectionCallback();

        final Intent exlIntent = createExplicitIntentForSpecifiedService();
        if(exlIntent != null){
            mContext.bindService(exlIntent,mConnectionCallback,Context.BIND_AUTO_CREATE);
            Log.d(TAG,"initTelePhone() bind TelePhoneService");
        }else{
            Log.d(TAG,"initTelePhone() please check why TelePhoneService not existed!!");
        }
    }

    private Intent createExplicitIntentForSpecifiedService() {
        final Intent intent = new Intent();
        intent.setAction(INTENT_TELEPHONE_SERVICE);
        intent.addCategory(INTENT_TELEPHONE_CATEGORY);
        final Intent explicitIntent = new Intent(createExplicitFromImplicitIntent(mContext, intent));
        Log.d(TAG, "createExplicitIntentForSpecifiedService explicitIntent");
        return explicitIntent;
    }

    private Intent createExplicitFromImplicitIntent(Context context, Intent implicitIntent) {
        // Retrieve all services that can match the given intent
        PackageManager pm = context.getPackageManager();
        List<ResolveInfo> resolveInfo = pm.queryIntentServices(implicitIntent,0);

        // Make sure only one match was found
        if (resolveInfo == null || resolveInfo.size() != 1) {
            Log.d(TAG, "createExplicitFromImplicitIntent failed, please check!!!!!!!");
            return null;
        }

        // Get component info and create ComponentName
        ResolveInfo serviceInfo = resolveInfo.get(0);
        String packageName = serviceInfo.serviceInfo.packageName;
        String className = serviceInfo.serviceInfo.name;
        ComponentName component = new ComponentName(packageName, className);

        // Create a new intent. Use the old one for extras and such reuse
        Intent explicitIntent = new Intent(implicitIntent);

        // Set the component to be explicit
        explicitIntent.setComponent(component);

        return explicitIntent;
    }

    private class ConnectionCallback implements ServiceConnection {

        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            Log.d(TAG,"ConnectionCallback onServiceConnected()");
            mTelePhoneBinder = ITelePhoneBinder.Stub.asInterface(service);
            try{
                mTelePhoneBinder.registerTelePhoneCallback(mTelePhoneCallback);
            }catch (RemoteException e){
                e.printStackTrace();
            }
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            Log.d(TAG,"ConnectionCallback onServiceDisconnected()");
            try{
                mTelePhoneBinder.unregisterTelePhoneCallback(mTelePhoneCallback);
                mTelePhoneBinder = null;
            }catch (RemoteException e){
                e.printStackTrace();
            }
        }
    }

    private final class TelePhoneListener extends ITelePhoneCallback.Stub{

        @Override
        public void onCallState(int state) throws RemoteException {
            Log.d(TAG,"onCallState() state ="+state);
        }

        @Override
        public void OnSuccess(int state) throws RemoteException {
            Log.d(TAG,"OnSuccess() state ="+state);
        }

        @Override
        public void onFailure(int state) throws RemoteException {
            Log.d(TAG,"onFailure() state ="+state);
        }
    }

    public int startCall(String callnumber)
    {
        int ret = -1;
        Log.d(TAG,"startCall() callnumber ="+callnumber);
        //SysProperties.setAudioVirtualState(mContext,false);
        try{
            if(mTelePhoneBinder != null){
                ret = mTelePhoneBinder.startCall(callnumber);
            }
        }catch (RemoteException e){
            e.printStackTrace();
        }
        return ret;
    }

    public int stopCall(String callnumber)
    {
        int ret = -1;
        Log.d(TAG,"stopCall() callnumber ="+callnumber);
        try{
            if(mTelePhoneBinder != null){
                ret = mTelePhoneBinder.stopCall(callnumber);
            }
        }catch (RemoteException e){
            e.printStackTrace();
        }
        //SysProperties.setAudioVirtualState(mContext,true);
        return ret;
    }

    public int getCallState()
    {
        int ret = -1;
        Log.d(TAG,"getCallState()");
        try{
            if(mTelePhoneBinder != null){
                ret = mTelePhoneBinder.getCallState();
            }
        }catch (RemoteException e){
            e.printStackTrace();
        }
        return ret;
    }

    public void onDestory()
    {
        SysProperties.setAudioVirtualState(mContext,false);
    }
}