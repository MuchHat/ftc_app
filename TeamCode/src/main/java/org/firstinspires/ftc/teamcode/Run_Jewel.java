package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

//********************************* MAIN OP CLASS ************************************************//

public class Run_Jewel {

    //********************************* VARIABLES *********************************************//

    boolean foundBlue = false;
    boolean foundRed = false;
    Team_Hardware_V9 robot = null;
    double speedIncrease = 1.5;

    //********************************* END VARIABLE *******************************************//

    void init(Team_Hardware_V9 aRobot) {

        robot = aRobot;
        robot.colorSensor.enableLed(true);

        robot.closeClawAuto();
        robot.moveArmPosZero();
        waitMillis(66);
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
        // KNOCK THE JEWEL IF COLOR FOUND
        if (foundBlue || foundRed) {

            boolean knockFirst = true; //knock the ball in front or the other one

            if (blue_ && foundBlue) knockFirst = false;
            if (blue_ && !foundBlue) knockFirst = true;
            if (red_ && foundRed) knockFirst = false;
            if (red_ && !foundRed) knockFirst = true;

            //GO IN BETWEEN THE JEWELS
            robot.moveInches(2.5, 0.22, 2);
            waitMillis(33);

            //EXTEND ARM
            robot.moveArm(armKnockB, armKnockE);

            //KNOCK AND MOVE AT THE EDGE
            if (knockFirst) {
                if( red_ ){
                    robot.moveInches(-4, 0.22 * speedIncrease, 2);
                    robot.moveArmPosZero();
                    robot.moveInches(12+6.7, 0.33 * speedIncrease, 6); // should be in front of Vuforia
                    waitMillis(333); // should be in front of Vuforia
                }
                if( blue_){
                    robot.moveInches(-4, 0.22 * speedIncrease, 2);
                    robot.moveArmPosZero();
                    robot.moveInches(7, 0.22 * speedIncrease, 6); // should be in front of Vuforia
                    waitMillis(333); // should be in front of Vuforia
                    robot.moveInches(-12.25-6.7, 0.22 * speedIncrease, 6);
                }
            }
            if (!knockFirst) {
                if( red_ ){
                    robot.moveInches(7.5, 0.22 * speedIncrease, 1);
                    robot.moveArmPosZero(); // should be in front of Vuforia
                    robot.moveInches(6.7, 0.22 * speedIncrease, 1);
                }
                if (blue_) {
                    robot.moveInches(4, 0.22 * speedIncrease, 2);
                    robot.moveArmPosZero(); // should be in front of Vuforia
                    robot.moveInches(-12-6.7, 0.33 * speedIncrease, 6);
                }
            }
        }
        // IF COLOR NOT FOUND JUST MOVE TO EDGE
        if (!foundBlue && !foundRed) {

            robot.moveArmPosZero();
            robot.showTeamColor();
            if (red_) robot.moveInches(6.5+6.7, 0.33 * speedIncrease, 1);
            if (blue_) robot.moveInches(-6.5-6.7, 0.33 * speedIncrease, 1);
        }
        //TURN OFF LED
        robot.moveArmPosZero();
        robot.colorSensor.enableLed(false);
        robot.showTeamColor();
        waitMillis(11);

        //FIX HEADING
        //robot.turnTo12();
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
    }
}

