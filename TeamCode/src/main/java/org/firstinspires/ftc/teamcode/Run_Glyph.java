package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static android.os.SystemClock.sleep;

//********************************* MAIN OP CLASS ************************************************//

public class Run_Glyph {

    //********************************* HW VARIABLES *********************************************//
    Team_Hardware_V9 robot = null;
    //********************************* CONSTANTS ************************************************//
    Boolean blueTeam = true;
    Boolean shortField = true;
    private Vu vu = new Vu();
    private HardwareMap hwMap = null;

    void init(Team_Hardware_V9 aRobot, HardwareMap aHwMap, boolean aBlueTeam, boolean aShortField) {

        hwMap = aHwMap;
        robot = aRobot;

        blueTeam = aBlueTeam;
        shortField = aShortField;

        robot.rightClaw.setPosition(0.59);
        robot.leftClaw.setPosition(0.54);

        vu.init(hwMap);

    }

    void run() {

        if (robot == null) {
            return;
        }

        int[] moveDistance = {580, 358, 161};

        if (vu.targetSeen()) {
            robot.colorBeacon.green();
        } else {
            robot.colorBeacon.yellow();
        }

        robot.moveInches(178 / 24.5, 0.33);
        waitMillis(333);
        if (vu.targetSeen()) {
            robot.colorBeacon.green();
        } else {
            robot.colorBeacon.yellow();
        }
        waitMillis(1000);
        if (vu.targetSeen()) {
            robot.colorBeacon.green();
        } else {
            robot.colorBeacon.yellow();
        }

        robot.moveInches(432 / 24.5, 0.33);
        waitMillis(333);
        if (vu.targetSeen()) {
            robot.colorBeacon.green();
        } else {
            robot.colorBeacon.yellow();
        }

        robot.moveInches(-260 / 24.5, 0.33);
        waitMillis(333);
        robot.turnTo12();
        waitMillis(1000);

        int index = 0;
        if (vu.targetSeen()) {
            robot.colorBeacon.green();
            index = vu.lastTargetSeenNo;
            robot.colorBeacon.green();
        } else {
            index = 2; //go midedle if no vuforia
            robot.colorBeacon.yellow();
        }
        waitMillis(111);

        if (index != 0) {
            robot.moveInches(moveDistance[index - 1] / 24.5, 0.22);
        }
        waitMillis(222);

        if (vu.targetSeen()) {
            robot.colorBeacon.green();

            int errDis = 15; // 1 left 2 center 3 right
            int[] xValues = {460, 415, 340}; // Desired value for left, right, and middle
            index = vu.getLastTargetSeenNo() - 1;
            int desiredX = xValues[index];

            waitMillis(222);
            robot.moveInches(50 / 24.5, 0.33);

            double vuX = vu.getX();
            double attempts = 0;

            if (vu.getX() - desiredX > 0) { //TODO
                while (vu.getX() - desiredX > errDis) {
                    robot.colorBeacon.purple();
                    robot.moveInches(-10 / 24.5, 0.15);
                }
            } else {
                while (vu.getX() - desiredX < errDis) {
                    robot.colorBeacon.purple();
                    robot.moveInches(10 / 24.5, 0.15);
                }
            }

            if (blueTeam)
                robot.colorBeacon.blue();
            else
                robot.colorBeacon.red();
        }
        //turn to put the glyph in

        robot.colorBeacon.teal();
        robot.turnTo3();

        if (blueTeam)
            robot.colorBeacon.blue();
        else
            robot.colorBeacon.red();
        waitMillis(222);

        robot.stopRobot();
        waitMillis(222);

        //put the glyph in
        robot.moveInches(0.5 * 11, 0.33);

        robot.rightClaw.setPosition(0.75);
        robot.leftClaw.setPosition(0.22);
        waitMillis(111);

        robot.moveInches(-100 / 24.5, 0.22);
        robot.moveInches(3, 0.22);

        robot.stopRobot();

        //********************************* END LOOP *****************************************//
    }

    private void waitMillis(double millis) {

        sleep((long) millis);
//        millis = Range.clip(millis, 0.01, millis);
//        runtimeWait.reset();
//        while (runtimeWait.nanoseconds() < millis * 1000 * 1000) {
//            idle();
//        }

    }
}

