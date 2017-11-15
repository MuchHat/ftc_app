package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

//********************************* MAIN OP CLASS ************************************************//

public class Run_Jewel {

    boolean foundBlue = false;
    boolean foundRed = false;

    //********************************* HW VARIABLES *********************************************//
    Team_Hardware_V9 robot = null;

    //********************************* CONSTANTS ************************************************//

    void init(Team_Hardware_V9 aRobot) {

        robot = aRobot;
        robot.colorSensor.enableLed(true);

        robot.closeClawAuto();
        robot.moveArmPosZero();
        waitMillis(66);
    }

    void run() {

        if (robot == null) {
            return;
        }
        int findPositions = 4;

        double armFindJewelB[] = {0.19, 0.14, 0.10, 0.07};
        double armFindJewelE[] = {0.64, 0.69, 0.78, 0.82};

        double armKnockB = 0;
        double armKnockE = 0.90;

        double armExtendedB = 0.13;
        double armExtendedE = 0.65;

        //ARM IN EXTENDED POSITION
        robot.colorBeacon.purple();
        robot.moveArm(armExtendedB, armExtendedE);
        waitMillis(11);

        //GO THRU POSITIONS TO FIND THE JEWEL
        for (int i = 0; i < findPositions; i++) {
            double crrBase = armFindJewelB[i];
            double crrElbow = armFindJewelE[i];

            robot.colorBeacon.purple();
            robot.moveArm(crrBase, crrElbow);
            waitMillis(111);

            if (foundJewel()) {
                robot.moveArm(armExtendedB, armExtendedE);
                waitMillis(11);
                break;
            }
        }
        // KNOCK THE BALL IF COLOR FOUND
        if (foundBlue || foundRed) {

            boolean knockFirst = true; //knock the ball in front or the other one

            if (robot.blueTeam && foundBlue) knockFirst = false;
            if (robot.blueTeam && !foundBlue) knockFirst = true;
            if (!robot.blueTeam && foundRed) knockFirst = false;
            if (!robot.blueTeam && !foundRed) knockFirst = true;

            //GO IN BETWEEN THE JEWELS
            robot.moveInches(2.5, 0.22, 2);
            waitMillis(11);

            //EXTEND ARM
            robot.moveArm(armKnockB, armKnockE);

            //IF COLOR FOUND KNOCK AND MOVE TO EDGE
            if (!knockFirst) {
                robot.moveInches(4, 0.22, 1);
                robot.moveArmPosZero();
                if (robot.blueTeam) {
                    waitMillis(11);
                    robot.moveInches(-6.5 * 2, 0.44, 1);
                }
            } else {
                robot.moveInches(-4, 0.22, 1);
                robot.moveArmPosZero();
                if (!robot.blueTeam) {
                    waitMillis(11);
                    robot.moveInches(8, 0.44, 1);
                } else {
                    waitMillis(11);
                    robot.moveInches(-4, 0.44, 1);
                }
            }
            waitMillis(66);
        } else {
            // IF COLOR NOT FOUND JUST MOVE TO EDGE
            robot.moveArmPosZero();
            robot.showTeamColor();
            if (!robot.blueTeam) {
                robot.moveInches(6.5, 0.66, 1);
            } else robot.moveInches(-6.5, 0.66, 1);
            waitMillis(11);
        }

        //TURN OFF LED
        robot.moveArmPosZero();
        robot.colorSensor.enableLed(false);
        robot.showTeamColor();
        waitMillis(11);

        //FIX HEADING
        robot.turnTo12();
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

