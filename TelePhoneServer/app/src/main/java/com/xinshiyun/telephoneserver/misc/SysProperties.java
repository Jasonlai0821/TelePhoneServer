package com.xinshiyun.telephoneserver.misc;

import android.content.Context;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
//import android.os.SystemProperties;

public class SysProperties {

    // 获取机芯
    public static String getChip(Context contex) {
        return SystemPropertiesProxy.get(contex, "ro.build.skymodel", "2C01");
//        return SystemProperties.get("ro.build.skymodel", "31P23");
    }

    // 获取机型
    public static String getModel(Context contex) {
//        return "null";
        return SystemPropertiesProxy.get(contex, "ro.build.skytype", "AIUI98C");
//        return SystemProperties.get("ro.build.skytype", "AIUI98");
    }

    public static String getSubModel(Context context) {
        return SystemPropertiesProxy.get(context, "ro.build.update.skytype", "SBL3S");
    }

    // 获取当前系统版本号
    public static String getVersion(Context contex) {
        return SystemPropertiesProxy.get(contex, "ro.build.skyversion", "000.000.000");
//        return removeDot(SystemProperties.get("ro.build.skyversion", "015.010.240"));
    }

    private static String removeDot(String s) {
        if (s.contains(".")) {
            return s.replace(".", "");
        }
        return s;
    }

    // 获取本机的mac地址
    public static String getMac(Context context) {
        WifiManager wifi = (WifiManager) context.getSystemService(Context.WIFI_SERVICE);
        WifiInfo info = wifi.getConnectionInfo();
        if (info.getMacAddress() == null) {
            return "acd074d89d6f";
        }
        return stringFormat(info.getMacAddress());
    }

    private static String stringFormat(String mac) {
        StringBuilder builder = new StringBuilder();
        String[] res = mac.split(":");
        for (String s : res) {
            builder.append(s);
        }
        return builder.toString();
    }

    public static boolean getAudioVirtualState(Context context)
    {
        return SystemPropertiesProxy.getBoolean(context,"persist.sys.audio.streamtype.virtual",false);
    }

    public static void setAudioVirtualState(Context context,boolean state)
    {
        if(state == true){
            SystemPropertiesProxy.set(context,"persist.sys.audio.streamtype.virtual","true");
        }else{
            SystemPropertiesProxy.set(context,"persist.sys.audio.streamtype.virtual","false");
        }
    }
}
