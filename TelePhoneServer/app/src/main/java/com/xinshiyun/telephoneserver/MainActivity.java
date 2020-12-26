package com.xinshiyun.telephoneserver;

import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import androidx.appcompat.app.AppCompatActivity;
import com.xinshiyun.telephoneserver.linphone.PhoneVoiceUtils;

public class MainActivity extends AppCompatActivity implements View.OnClickListener {
    private static String TAG = MainActivity.class.getSimpleName();
    private Button tvCall;
    private Button tvRegistered;
    private Button tvHangUp;
    private Button tvUnRegistered, tvStartRecorder, tvStopRecorder;
    private EditText et_number;
    private static TelePhoneManager mTelePhoneManager = null;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        tvCall = findViewById(R.id.tv_call);
        tvRegistered = findViewById(R.id.tv_registered);
        tvHangUp = findViewById(R.id.tv_hangUp);
        tvUnRegistered = findViewById(R.id.tv_unRegistered);
        tvStartRecorder = findViewById(R.id.tv_start_recorder);
        tvStopRecorder = findViewById(R.id.tv_stop_recorder);
        et_number = findViewById(R.id.et_number);
        tvCall.setOnClickListener(this);
        tvRegistered.setOnClickListener(this);
        tvHangUp.setOnClickListener(this);
        tvUnRegistered.setOnClickListener(this);
        tvStartRecorder.setOnClickListener(this);
        tvStopRecorder.setOnClickListener(this);

        mTelePhoneManager = new TelePhoneManager(this);
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.tv_registered:
//                LinphoneManager.createAndStart(MainActivity.this, mCoreListnerStub);
                break;
            case R.id.tv_unRegistered:
                break;
            case R.id.tv_call:
                if(mTelePhoneManager != null){
                    mTelePhoneManager.startCall(et_number.getText().toString());
                }
                break;
            case R.id.tv_hangUp:
                if(mTelePhoneManager != null){
                    mTelePhoneManager.stopCall(et_number.getText().toString());
                }
                Log.d(TAG, "isRegistered:" + PhoneVoiceUtils.getInstance().isRegistered());
                break;
            case R.id.tv_start_recorder:
                //startService(new Intent(this, AudioRecorderService.class));
//                startRecorder();
                break;
            case R.id.tv_stop_recorder:
                //mAudioRecorderService.stopRecord();
//                doStop();
                break;
        }
    }

    @Override
    public void onDestroy() {
        Log.d(TAG, "onDestroy()");
        super.onDestroy();
        mTelePhoneManager.onDestory();
        mTelePhoneManager = null;
    }
}




