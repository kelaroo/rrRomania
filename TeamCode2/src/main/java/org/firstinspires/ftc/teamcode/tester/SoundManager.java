package org.firstinspires.ftc.teamcode.tester;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

public class SoundManager {
    SoundPlayer sound;
    HardwareMap hardwareMap;

    public Map<String, Integer> idMap = new HashMap<String, Integer>();

    public SoundManager(HardwareMap hw) {
        sound = SoundPlayer.getInstance();
        hardwareMap = hw;
    }

    public boolean addFile(String fileName) {
        if(idMap.containsKey(fileName))
            return false;
        Integer currID = hardwareMap.appContext.getResources().getIdentifier(
                fileName, "raw", hardwareMap.appContext.getPackageName());

        if(currID == 0)
            return false;

        sound.preload(hardwareMap.appContext, currID);

        idMap.put(fileName, currID);

        return true;
    }

    public boolean playSound(String s) {
        if(!idMap.containsKey(s))
            return false;
        int id = idMap.get(s);

        sound.startPlaying(hardwareMap.appContext, id);

        return true;
    }

    public void stopSounds() {
        sound.stopPlayingAll();
    }
}
