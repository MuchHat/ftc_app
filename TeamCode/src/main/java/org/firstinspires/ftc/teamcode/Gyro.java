package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by MuchHat on 11/2/2017.
 */

public class Gyro {

    boolean isAvailable = false;
    double lastX = 0;
    double lastY = 0;
    HardwareMap hardwareMap = null;

    void init(HardwareMap aHwMap) {
        hardwareMap = aHwMap;
    }

    boolean available(){
        return isAvailable;
    }

    double getHeading(){
        return 0;
    }

    double getX(){
        return 0;
    }

    double getY(){
        return 0;
    }
}