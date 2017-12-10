package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

//********************************* MAIN OP CLASS ************************************************//

public class Run_Jewel {

    //********************************* VARIABLES *********************************************//

    Team_Hardware_V9 robot = null;
    double speedIncrease = 1.5;

    //********************************* END VARIABLE *******************************************//

    void init(Team_Hardware_V9 aRobot) {

        robot = aRobot;
        robot.colorSensor.enableLed(true);
    }

    void run() {

        boolean red_ = !robot.blueTeam;
        boolean blue_ = robot.blueTeam;
        boolean short_ = robot.shortField;
        boolean long_ = !robot.shortField;

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

        //START ARM IN EXTENDED POSITION
        robot.colorBeacon.purple();
        robot.moveArm(armExtendedB, armExtendedE);

        //GO THRU POSITIONS TO FIND THE JEWEL
        for (int a = 0; a < 3; a++) {

            if (a == 1) {
                // move back
                robot.moveInches(-0.3, 0.11, 1);
            }
            if (a == 2) {
                // move front
                robot.moveInches(0.6, 0.11, 1);
            }

            boolean found = false;
            for (int i = 0; i < findPositions; i++) {

                double crrBase = armFindJewelB[i];
                double crrElbow = armFindJewelE[i];

                robot.colorBeacon.purple();
                robot.moveArmStopOnJewelFound(crrBase, crrElbow);
                waitMillis(33);

                if (robot.foundRed || robot.foundBlue) {
                    found = true;
                    break;
                }
            }
            robot.moveArm(armExtendedB, armExtendedE);
            if (found) {
                break;
            }
        }
        // KNOCK THE JEWEL IF A COLOR FOUND
        if (robot.foundRed || robot.foundBlue) {

            boolean knockFirst = true; //knock the ball in front or the other one

            if (blue_ && robot.foundBlue) knockFirst = false;
            if (blue_ && !robot.foundBlue) knockFirst = true;
            if (red_ && robot.foundRed) knockFirst = false;
            if (red_ && !robot.foundRed) knockFirst = true;

            //GO IN BETWEEN THE JEWELS
            if (knockFirst) robot.moveInches(2, 0.19, 2);
            if (!knockFirst) robot.moveInches(3, 0.19, 2);
            waitMillis(11);

            //EXTEND ARM
            robot.moveArm(armKnockB, armKnockE);

            //KNOCK AND MOVE AT THE EDGE
            if (knockFirst) {
                if (red_) {
                    robot.moveInches(-3, 0.44, 2);
                    robot.moveArm(armExtendedB, armExtendedE);
                    robot.moveInches(6.7, 0.66, 8); // should be in front of Vuforia
                    robot.moveArmPosZero();
                    waitVu(1111); // should be in front of Vuforia
                    robot.moveLinearStopOnFlatRampDownInches = 6;
                    robot.moveLinearGyroTrackingEnabled = true;
                    robot.moveLinearGyroHeadingToTrack = 0;
                    robot.moveInchesStopOnFlat(12, 0.55, 8); // should be in front of Vuforia
                }
                if (blue_) {
                    robot.moveInches(-3, 0.33 * speedIncrease, 2);
                    robot.moveArm(armExtendedB, armExtendedE);
                    robot.moveInches(5.5, 0.33 * speedIncrease, 6); // should be in front of Vuforia
                    robot.moveArmPosZero();
                    waitVu(1111); // should be in front of Vuforia
                    robot.moveLinearStopOnFlatRampDownInches = 8;
                    robot.moveLinearGyroTrackingEnabled = true;
                    robot.moveLinearGyroHeadingToTrack = 0;
                    robot.moveInchesStopOnFlat(-12.25 - 6.7 - 3, 0.55, 8);
                }
            }
            if (!knockFirst) {
                if (red_) {
                    robot.moveInches(6.5, 0.33 * speedIncrease, 3);
                    robot.moveArmPosZero(); // should be in front of Vuforia
                    waitVu(1111);
                    robot.moveLinearStopOnFlatRampDownInches = 6;
                    robot.moveLinearGyroTrackingEnabled = true;
                    robot.moveLinearGyroHeadingToTrack = 0;
                    robot.moveInchesStopOnFlat(6.7, 0.33 * speedIncrease, 3);
                }
                if (blue_) {
                    robot.moveInches(3, 0.22 * speedIncrease, 3);
                    robot.moveArmPosZero(); // should be in front of Vuforia
                    waitVu(1111);
                    robot.moveLinearStopOnFlatRampDownInches = 8;
                    robot.moveLinearGyroTrackingEnabled = true;
                    robot.moveLinearGyroHeadingToTrack = 0;
                    robot.moveInchesStopOnFlat(-12 - 5.7 - 5, 0.33 * speedIncrease, 8);
                }
            }
        }
        // IF COLOR NOT FOUND JUST MOVE TO EDGE
        if (!robot.foundBlue && !robot.foundRed) {

            if (red_) {
                robot.moveInches(6.5 + 3, 0.33 * speedIncrease, 3);
                robot.moveArmPosZero(); // should be in front of Vuforia
                waitVu(1111);
                robot.moveLinearStopOnFlatRampDownInches = 6;
                robot.moveLinearGyroTrackingEnabled = true;
                robot.moveLinearGyroHeadingToTrack = 0;
                robot.moveInchesStopOnFlat(6.7, 0.33 * speedIncrease, 3);
            }
            if (blue_) {
                robot.moveInches(3, 0.22 * speedIncrease, 3);
                robot.moveArmPosZero(); // should be in front of Vuforia
                waitVu(1111);
                robot.moveLinearStopOnFlatRampDownInches = 6;
                robot.moveLinearGyroTrackingEnabled = true;
                robot.moveLinearGyroHeadingToTrack = 0;
                robot.moveInchesStopOnFlat(-12 - 5.7 - 3 -3, 0.33 * speedIncrease, 8);
            }
        }

        //TURN OFF LED
        robot.colorSensor.enableLed(false);

        // ROBOT SHOULD BE ON FLAT BACK AGAINST PLATFORM
    }

    void waitVu(double millis) {

        for (int i = 0; i < (int)(millis / 6); i++) {
            if (robot.vu.targetSeen()) {
                robot.colorBeacon.green();
                return;
            }
            waitMillis(11);
        }
    }


    private void waitMillis(double millis) {

        sleep((long) millis);
    }
}

