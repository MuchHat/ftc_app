package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

//********************************* MAIN OP CLASS ************************************************//

public class Run_Glyph {

    //********************************* HW VARIABLES *********************************************//
    Team_Hardware_V9 robot = null;
    private double secsLeftAtStart = 30;
    private ElapsedTime timer = new ElapsedTime();

    //********************************* CONSTANTS ************************************************//
    private Vu vu = new Vu();
    private HardwareMap hwMap = null;

    void init(Team_Hardware_V9 aRobot, HardwareMap aHwMap) {
        hwMap = aHwMap;
        robot = aRobot;
        vu.init(hwMap);
    }

    boolean noTimeLeft() {
        /*waitMillis(11);
        if ((secsLeftAtStart - timer.seconds()) > 1) {
            return false;
        }
        robot.stopRobot();*/
        return false;
    }

    void run(double secsLeft) {
        runSonar(secsLeft);
    }

    void runSonar(double secsLeft) {

        int columnIndex = 2;
        boolean red_ = !robot.blueTeam;
        boolean blue_ = robot.blueTeam;
        boolean short_ = robot.shortField;
        boolean long_ = !robot.shortField;

        if (robot == null) {
            return;
        }
        // START TIMER
        {
            secsLeftAtStart = secsLeft;
            timer.reset();
            if (noTimeLeft()) return;
        }
        // DETERMINE WHAT COLUMN
        {
            if (vu.targetSeen()) {
                columnIndex = vu.lastTargetSeenNo;
            } else {
                columnIndex = 2; //go middle if no vuforia
            }
            showIfTargetSeen();
            if (noTimeLeft()) return;
        }
        // MOVE OFF THE PLATFORM, MOVE STARTS AT EDGE
        {
            if (red_ && short_) robot.moveInches(11, 0.88, 4);
            if (red_ && long_) robot.moveInches(11, 0.88, 4);
            if (blue_ && short_) robot.moveInches(-11, 0.88, 4);
            if (blue_ && long_) robot.moveInches(-11, 0.88, 4);

            if (noTimeLeft()) return;
        }
        // BACK AGAINST THE PLATFORM
        {
            if (red_) robot.moveInches(-1, 0.22, 1);
            if (blue_) robot.moveInches(1, 0.22, 1);
            if (noTimeLeft()) return;
        }
        // MOVE IN FRONT OF BOX USING ENCODERS
        {
            if (red_ && long_) robot.moveInches(4, 0.88, 4);
            if (blue_ && long_) robot.moveInches(-4, 0.88, 4);

            double redShortEncoder[] = {24, 16, 8}; // L C R
            double redLongEncoder[] = {24, 16, 8}; // L C R
            double blueShortEncoder[] = {24, 16, 8}; // L C R
            double blueLongEncoder[] = {24, 16, 8}; // L C R
            double moveDistance = 8;

            if (red_ && short_) moveDistance = redShortEncoder[columnIndex - 1];
            if (red_ && long_) moveDistance = redLongEncoder[columnIndex - 1];
            if (blue_ && short_) moveDistance = blueShortEncoder[columnIndex - 1];
            if (blue_ && long_) moveDistance = blueLongEncoder[columnIndex - 1];

            if (red_ && short_) robot.moveInches(moveDistance, 0.22, 1);
            if (red_ && long_) robot.moveSideInches(moveDistance, 0.22, 1);
            if (blue_ && short_) robot.moveInches(-moveDistance, 0.22, 1);
            if (blue_ && long_) robot.moveSideInches(-moveDistance, 0.22, 1);
            if (noTimeLeft()) return;
        }
        // TURN TOWARDS THE BOX
        {
            if (red_ && short_) robot.turnTo3();
            if (red_ && long_) robot.turnTo12();
            if (blue_ && short_) robot.turnTo3();
            if (blue_ && long_) robot.turnTo6();
            if (noTimeLeft()) return;
        }
        // MOVE IN FRONT OF THE COLUMN USING SONAR
        {
            double redShortSonar[] = {0.249, 0.171, 0.117}; // L C R
            double redLongSonar[] = {0.36, 0.32, 0.28}; // L C R
            double blueShortSonar[] = {0.12, 0.18, 0.24}; // L C R
            double blueLongSonar[] = {0.36, 0.32, 0.28}; // L C R
            double columnSonarPos = 0.44;

            if (red_ && short_) columnSonarPos = redShortSonar[columnIndex - 1];
            if (red_ && long_) columnSonarPos = redLongSonar[columnIndex - 1];
            if (blue_ && short_) columnSonarPos = blueShortSonar[columnIndex - 1];
            if (blue_ && long_) columnSonarPos = blueLongSonar[columnIndex - 1];

            if (red_ && short_) robot.moveBySonarRight(columnSonarPos, 0.44, 6);
            if (red_ && long_) robot.moveBySonarRight(columnSonarPos, 0.44, 6);
            if (blue_ && short_) robot.moveBySonarLeft(columnSonarPos, 0.44, 6);
            if (blue_ && long_) robot.moveBySonarLeft(columnSonarPos, 0.44, 6);

            if (noTimeLeft()) return;
        }
        // TURN TOWARDS THE BOX
        {
            if (red_ && short_) robot.turnTo3();
            if (red_ && long_) robot.turnTo12();
            if (blue_ && short_) robot.turnTo3();
            if (blue_ && long_) robot.turnTo6();
            if (noTimeLeft()) return;
        }

        //PUT THE GLYPH IN
        {
            robot.moveInches(5.5, 0.22, 1);
            robot.openClawAuto();
            waitMillis(22);
            if (noTimeLeft()) return;
        }

        //BACKOFF
        {
            robot.moveInches(-4, 1, 1);
            robot.setClawPosZero();
            robot.moveInches(2, 1, 1);
            robot.moveInches(-4, 1, 1);
            robot.stopRobot();
        }

    }

    void runOld(double secsLeft) {
        if (robot == null) {
            return;
        }

        secsLeftAtStart = secsLeft;
        timer.reset();
        if (noTimeLeft()) return;

        double[] moveDistance = {15.4, 9, 2};
        double[] moveDistanceLongField = {36, 26, 5};

        //****  0. ADJUST VARIABLES DEPENDING ON THE FIELD **************************************//

        double direction = 1.0;
        if (robot.blueTeam) direction = -1.0;

        if (!robot.shortField) {
            for (int i = 0; i < moveDistance.length; i++) {
                moveDistance[i] = moveDistanceLongField[i];
            }
        }

        //****  1. WAIT IF NEEDED FOR VUFORIA TO LOCK ON THE TARGET ******************************//

        if (robot.blueTeam) {
            robot.moveInches(-6, 0.4, 0.88);
        }
        for (int i = 0; i < 22; i++) {
            if (vu.targetSeen()) {
                break;
            }
            robot.colorBeacon.white();
            waitMillis(66);
            if (noTimeLeft()) return;
        }
        showIfTargetSeen();

        //****  2. MOVE OFF THE PLATFORM *********************************************************//

        if (robot.blueTeam) {
            robot.moveInches(10 * direction, 0.6, 0.66);
        } else {
            robot.moveInches(13.5 * direction, 0.8, 0.66);
        }
        showIfTargetSeen();
        if (noTimeLeft()) return;

        //****  3. CORRECT HEADING IF NEEDED ********************** ******************************//

        robot.turnTo12();
        showIfTargetSeen();
        if (noTimeLeft()) return;

        //****  4. DETERMINE WHAT COLUMN  *********************************//
        int index = 0;
        if (vu.targetSeen()) {
            index = vu.lastTargetSeenNo;
            showIfTargetSeen();
        } else {
            index = 2; //go middle if no vuforia
        }
        if (noTimeLeft()) return;

        //****  5. MOVE IN FRONT OF THE BOX *********************************//

        if (!robot.shortField) {
            //****  5. LONG FIELD *******************************//

            robot.moveSideInches(moveDistanceLongField[index - 1] * direction,
                    0.44, 4);
            //TODO sonar

            /*
            if (!robot.blueTeam) {
                robot.moveSideBySonarRight(22, 0.22, 3);
            } else {
                robot.moveSideBySonarLeft(22, 0.22, 3);
            }
            */

        } else {
            //****  5. SHORT FIELD *******************************//

            if (index != 0) {
                robot.moveInches(moveDistance[index - 1] * direction, 0.88, 2);
                waitMillis(22);
            }
            if (noTimeLeft()) return;

            //****  5.1. TURN 90 TOWARDS THE BOX ******************************************************//

            if (robot.shortField) {
                robot.turnTo3();
            } else {
                if (robot.blueTeam) {
                    robot.turnTo6();
                } else {
                    robot.turnTo12();
                }
            }
            robot.stopRobot();
            waitMillis(22);

        }
        if (noTimeLeft()) return;

        //****  6. PUT THE GLYPH IN  *************************************************************//
        robot.moveInches(5.5, 0.22, 1);
        robot.openClawAuto();
        waitMillis(22);
        if (noTimeLeft()) return;

        //****  7. BACKOFF ***********************************************************************//
        robot.moveInches(-4, 1, 1);
        robot.setClawPosZero();
        robot.moveInches(2, 1, 1);
        robot.moveInches(-4, 1, 1);
        robot.stopRobot();

    }

    private void waitMillis(double millis) {

        sleep((long) millis);
    }

    void showIfTargetSeen() {
        if (vu.targetSeen()) {
            robot.colorBeacon.green();
        } else {
            robot.showTeamColor();
        }
    }
}

