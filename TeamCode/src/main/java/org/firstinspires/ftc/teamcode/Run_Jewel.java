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

        double armFindJewelB[] = {0.19, 0.14, 0.10, 0.07};
        double armFindJewelE[] = {0.64, 0.69, 0.78, 0.82};

        int findPositions = 4;
        double armKnockB = 0;
        double armKnockE = 0.90;

        double armExtendedB = 0.13;
        double armExtendedE = 0.65;

        //****  1. ARM IN EXTENDED POSITION ******************************************************//

        robot.colorBeacon.purple();
        robot.moveArm(armExtendedB, armExtendedE);
        waitMillis(11);

        //****   2. GO THRU POSITIONS TO FIND THE JEWEL *****************************************//

        for (int i = 0; i < findPositions; i++) {

            double crrBase = armFindJewelB[i];
            double crrElbow = armFindJewelE[i];

            robot.colorBeacon.purple();

            robot.moveArm(crrBase, crrElbow);
            waitMillis(66);

            if (foundJewel()) {
                foundPos = i;
                robot.moveArm(armExtendedB, armExtendedE);
                waitMillis(11);
                break;
            }
        }
        //****   KNOCK THE BALL IF COLOR FOUND ***************************************************//

        if (foundBlue || foundRed) {

            boolean knockFirst = true; //knock the ball in front or the other one

            if (robot.blueTeam && foundBlue) knockFirst = false;
            if (robot.blueTeam && !foundBlue) knockFirst = true;
            if (!robot.blueTeam && foundRed) knockFirst = false;
            if (!robot.blueTeam && !foundRed) knockFirst = true;

            //****   3. GO IN BETWEEN THE BALLS **************************************************//

            robot.moveInches(2.5, 0.22, 2);
            waitMillis(11);

            robot.moveArm(armKnockB, armKnockE);
            //****   4. KNOCK THE BALL ***********************************************************//

            if (!knockFirst) {
                //*****  MOVE FORWARD AND REMAIN AT EDGE *************************************//
                robot.moveInches(4, 0.44, 1);
                // if blue move back to get to the edge of the platform

                robot.moveArmPosZero();
                robot.showTeamColor();

                if (robot.blueTeam) {
                    waitMillis(11);
                    robot.moveInches(-6.5 * 2, 0.44, 1);
                }
            } else {
                //******  MOVE BACK THEN FW TO END UP AT EDGE ********************************//
                robot.moveInches(-4, 0.44, 1);
                //if red move fw to get to the edge of the platform

                robot.moveArmPosZero();
                robot.showTeamColor();

                if (!robot.blueTeam) {
                    waitMillis(11);
                    robot.moveInches(8, 0.88, 1);
                } else {
                    waitMillis(11);
                    robot.moveInches(-4, 0.88, 1);
                }
            }
            waitMillis(11);
        } else {
            robot.moveArmPosZero();
            robot.showTeamColor();
            if (!robot.blueTeam) {
                robot.moveInches(6.5, 0.88, 1);
            } else robot.moveInches(-6.5, 0.88, 1);
        }

        //****   6. PUT ARM BACK AT POS ZERO *************************************************//

        robot.moveArmPosZero();
        robot.colorSensor.enableLed(false);
        robot.showTeamColor();
        waitMillis(11);

        robot.turnTo12();
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

