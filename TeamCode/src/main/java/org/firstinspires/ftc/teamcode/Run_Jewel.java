package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

//********************************* MAIN OP CLASS ************************************************//

public class Run_Jewel {

    boolean foundBlue = false;
    boolean foundRed = false;
    int foundPos = 0;

    //********************************* HW VARIABLES *********************************************//
    Team_Hardware_V9 robot = null;

    //********************************* CONSTANTS ************************************************//
    Boolean blueTeam = true;
    Boolean shortField = true;
    Boolean loaded = false;

    void init(Team_Hardware_V9 aRobot, boolean aBlueTeam, boolean aShortField) {

        robot = aRobot;
        robot.moveArm(robot.armPosZero[0], robot.armPosZero[1]);

        shortField = aShortField;
        blueTeam = aBlueTeam;
    }

    void run() {

        if (robot == null) {
            return;
        }

        robot.colorBeacon.yellow();

        double armFindJewelB[] = {0.19, 0.14, 0.12, 0.10};
        double armFindJewelE[] = {0.64, 0.69, 0.76, 0.81};

        double armKnockB[] = {0, 0, 0, 0};
        double armKnockE[] = {0.92, 0.92, 0.92, 0.92};

        double armExtendedB = 0.13;
        double armExtendedE = 0.65;

        int findPositions = 4;
        robot.moveArm(armExtendedB, armExtendedE);
        waitMillis(222);

        for (int i = 0; i < findPositions; i++) {

            double crrBase = armFindJewelB[i];
            double crrElbow = armFindJewelE[i];

            robot.moveArm(crrBase, crrElbow);
            waitMillis(111);

            if (foundJewel()) {
                foundPos = i;
                robot.moveArm(armExtendedB, armExtendedE);
                waitMillis(222);
                break;
            }
        }

        // move the jewel
        if (foundBlue || foundRed) {

            boolean knockFirst = true; //knock the ball in front or the other one
            if (blueTeam && foundBlue) knockFirst = true;
            if (!blueTeam && foundRed) knockFirst = true;
            if (blueTeam && !foundBlue) knockFirst = false;
            if (!blueTeam && !foundRed) knockFirst = false;

            robot.move(20);

            double crrBase = armKnockB[foundPos];
            double crrElbow = armKnockE[foundPos];
            robot.moveArm(crrBase, crrElbow);
            waitMillis(222);

            if (!knockFirst) { // move to between the balls
                robot.move(40);
                waitMillis(400);
            }

            if (knockFirst) {
                robot.move(-40);
                waitMillis(400);
            }

            robot.moveArm(armExtendedB, armExtendedE);
            robot.moveArm(robot.armPosZero[0], robot.armPosZero[1]);
            waitMillis(222);


            robot.colorBeacon.green();

        } else if (!foundBlue || !foundRed) {
            robot.colorBeacon.yellow();
        }

        robot.stopRobot();
    }

    boolean foundJewel() {

        if (robot.colorSensor.red() > robot.colorSensor.blue()) {
            foundRed = true;
            foundBlue = false;
            robot.colorBeacon.red();
            return true;
        }
        if (robot.colorSensor.blue() > robot.colorSensor.red()) {
            foundBlue = true;
            foundRed = false;
            robot.colorBeacon.blue();
            return true;
        }

        return false;
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

