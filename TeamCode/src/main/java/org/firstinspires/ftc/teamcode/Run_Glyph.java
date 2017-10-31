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

        //****  1. WAIT IF NEEDED FOR VUFORIA TO LOCK ON THE TARGET ******************************//

        if (vu.targetSeen()) {
            robot.colorBeacon.green();
        } else {
            robot.colorBeacon.yellow();
            waitMillis(888);
        }
        if (vu.targetSeen()) {
            robot.colorBeacon.green();
        } else {
            robot.colorBeacon.yellow();
            waitMillis(888);
        }

        //****  2. MOVE OFF THE PLATFORM *********************************************************//

        robot.moveInches(432 / 24.5, 0.33);
        waitMillis(222);
        if (vu.targetSeen()) {
            robot.colorBeacon.green();
        } else {
            robot.colorBeacon.yellow();
        }

        //****  3. CORRECT HEADING IF NEEDED ********************** ******************************//

        robot.turnTo12();
        waitMillis(222);

        //****  4. BACK AGAINST THE PLATFORM TO START FROM A KNOWN POS ***************************//

        robot.moveInches(-260 / 24.5, 0.33);
        waitMillis(222);

        //****  5. MOVE IN FRONT OF THE BOX L/M/R PER THE WUMARK *********************************//

        int index = 0;
        if (vu.targetSeen()) {
            robot.colorBeacon.green();
            index = vu.lastTargetSeenNo;
            robot.colorBeacon.green();
            robot.beaconBlink(index+1);
        } else {
            index = 2; //go midedle if no vuforia
            robot.colorBeacon.yellow();
        }
        waitMillis(222);

        if (index != 0) {
            robot.moveInches(moveDistance[index - 1] / 24.5, 0.22);
        }
        waitMillis(222);

        //****  6. FINE ADJUST THE POS USING VUMARK AS AN ANCHOR *********************************//

        if (vu.targetSeen()) {
            robot.colorBeacon.green();

            int errDis = 15; // 1 left 2 center 3 right
            int[] xValues = {460, 415, 340}; // Desired value for left, right, and middle
            index = vu.getLastTargetSeenNo() - 1;
            int desiredX = xValues[index];

            waitMillis(222);
            robot.moveInches(50 / 24.5, 0.33); //TODO

            double vuX = vu.getX();
            double attempts = 0;

            if (vu.getX() - desiredX > 0 && attempts < 11) { //TODO
                while (vu.getX() - desiredX > errDis) {
                    robot.colorBeacon.purple();
                    robot.moveInches(-10 / 24.5, 0.15);
                    attempts++;
                }
            } else {
                while (vu.getX() - desiredX < errDis && attempts < 11) {
                    robot.colorBeacon.purple();
                    robot.moveInches(10 / 24.5, 0.15);
                    attempts++;
                }
            }
        }

        //****  7. TURN 90 TOWARDS THE BOX ******************************************************//

        robot.colorBeacon.teal();
        robot.turnTo3();

        if (blueTeam)
            robot.colorBeacon.blue();
        else
            robot.colorBeacon.red();
        waitMillis(222);

        robot.stopRobot();
        waitMillis(222);

        //****  8. PUT THE GLYPH IN  *************************************************************//

        robot.moveInches(6.5, 0.33);

        robot.rightClaw.setPosition(0.75);
        robot.leftClaw.setPosition(0.22);
        waitMillis(111);

        //****  9. BACKOFF ***********************************************************************//

        robot.moveInches(-4, 0.22);

        //****  10. TURN 180 AND TUCK IT IN ******************************************************//

        robot.turnTo9();
        robot.moveInches(-2, 0.22);

        //****  11. MOVE 1 INCH AWAY FROM BOX FOR THE REST POSITION ******************************//

        robot.moveInches(1, 0.22);
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

