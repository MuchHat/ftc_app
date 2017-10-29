package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by gigela on 10/27/2017.
 */

public class Vu {

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;
    boolean targetWasVisible = false;
    double lastX = 0;
    double lastY = 0;
    int lastTargetSeenNo = 0;
    HardwareMap hardwareMap = null;

    void vuController() {

    }

    void init(HardwareMap aHwMap) {

        hardwareMap = aHwMap;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Ac6cr63/////AAAAGUsQTEyyG0kggwF13U8WoMlPgXZiUoKR9pf2nlfhVVfvDXFsTn0wufoywxzibq+y5BGBa2cChKWAcUkKaD9/ak5lwCm9Wp3Osk9omsMR0YYoxt4TuPktrflK4HuTH8cOAQA8YDuOs/SO/cgOmWbQZtRXN/lFkUwGZA9eiV5D8730BG2SBLPR4A9rcFs0Fp/yPgcm4Zsh5Kv2Ct8XjJXmXk5mAjERZ5B6hKQzf/4wd9tSQ6BeQLvsgd5nI0Pj+K1NHI4EyHdFyxCPu91AMcCsXCLjkABfYt11Zhxu1uYaFF/AcN3eBHRwprVpDEBBXOMnD4BRCj0xxYYPWWO6g4gcjqBPgBos5nCDk43KipEeX22z";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = true; //TODO
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
    }

    boolean targetVisible() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            targetWasVisible = true;
            lastTargetSeenNo = 0;

            if(vuMark == RelicRecoveryVuMark.LEFT)lastTargetSeenNo = 1;
            if(vuMark == RelicRecoveryVuMark.CENTER)lastTargetSeenNo = 2;
            if(vuMark == RelicRecoveryVuMark.RIGHT)lastTargetSeenNo = 3;

            return true;
        }

        return false;
    }

    boolean targetSeen() {
        if (targetWasVisible) return true;

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            targetWasVisible = true;
            lastTargetSeenNo = 0;

            if(vuMark == RelicRecoveryVuMark.LEFT)lastTargetSeenNo = 1;
            if(vuMark == RelicRecoveryVuMark.CENTER)lastTargetSeenNo = 2;
            if(vuMark == RelicRecoveryVuMark.RIGHT)lastTargetSeenNo = 3;

            return true;
        }

        return false;
    }

    double getX() {
        if (!targetSeen()) return 0;

        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
        VectorF trans = pose.getTranslation();

        lastX = trans.get(0);

        return lastX;
    }

    double getY() {
        if (!targetSeen()) return 0;

        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
        VectorF trans = pose.getTranslation();

        lastY = trans.get(1);

        return lastY;
    }

    int getLastTargetSeenNo(){

        return lastTargetSeenNo;
    }
}
