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

        //****  1. ARM IN EXTENDED POSITION ******************************************************//

        int findPositions = 4;
        robot.moveArm(armExtendedB, armExtendedE);
        waitMillis(111);

        //****   2. GO THRU $ POSITION TO FIND THE JEWEL *****************************************//

        for (int i = 0; i < findPositions; i++) {

            double crrBase = armFindJewelB[i];
            double crrElbow = armFindJewelE[i];

            robot.moveArm(crrBase, crrElbow);
            waitMillis(111);

            if (foundJewel()) {
                foundPos = i;
                robot.moveArm(armExtendedB, armExtendedE);
                waitMillis(111);
                break;
            }
        }

        //****   3. GO IN BETWEEN THE BALLS ******************************************************//

        // move the jewel
        if (foundBlue || foundRed) {

            robot.colorBeacon.purple();

            boolean knockFirst = true; //knock the ball in front or the other one

            if (blueTeam && foundBlue) knockFirst = true;
            if (!blueTeam && foundRed) knockFirst = true;
            if (blueTeam && !foundBlue) knockFirst = false;
            if (!blueTeam && !foundRed) knockFirst = false;

            // move between the balls
            robot.moveInches(3.5, 0.22);

            double crrBase = armKnockB[foundPos];
            double crrElbow = armKnockE[foundPos];
            robot.moveArm(crrBase, crrElbow);
            waitMillis(111);

            //*****  4. MOVE BACK OR FW TO KNOCKK THE CORRECT BALL *******************************//
            //*****  5. AFTER KNOCK MOVE SUCH ROBOT END UP AT THE EDGE OF PLATFORM ***************//
            //******************WITH CAMERA FACING THE VUMARK ************************************//

            if (!knockFirst) {
                robot.moveInches(4, 0.22);
                waitMillis(111);
                robot.moveArm(armExtendedB, armExtendedE);
            } else {
                robot.moveInches(-4, 0.22);
                waitMillis(111);
                robot.moveArm(armExtendedB, armExtendedE);
                waitMillis(111);
                robot.moveInches(8, 0.22);
            }

            //****   6. PUT ARM BACK AT POS ZERO *************************************************//

            robot.moveArm(armExtendedB, armExtendedE);
            robot.moveArmPosZero();
            waitMillis(111);
        }

        //***************************************************************************************//

        if (blueTeam) robot.colorBeacon.blue();
        else robot.colorBeacon.red();

        robot.beaconBlink(3);

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

