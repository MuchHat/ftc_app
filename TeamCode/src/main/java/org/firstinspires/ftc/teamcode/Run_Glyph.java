package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;

//********************************* MAIN OP CLASS ************************************************//

public class Run_Glyph {

    //********************************* VARIABLES *********************************************//
    Team_Hardware_V9 robot = null;
    double secsLeftAtStart = 30;
    ElapsedTime timer = new ElapsedTime();
    Vu vu = new Vu();
    HardwareMap hwMap = null;
    double speedIncrease = 1.8;

    //********************************* END VARIABLE ********************************************//

    void init(Team_Hardware_V9 aRobot, HardwareMap aHwMap) {
        hwMap = aHwMap;
        robot = aRobot;
        vu.init(hwMap);
    }

    boolean noTimeLeft(double secBuffer) {
        waitMillis(6);
        if ((secsLeftAtStart - timer.seconds()) > secBuffer) {
            return false;
        }
        robot.stopRobot();
        return true;
    }

    boolean timeLeft(double secBuffer) {

        return !noTimeLeft(secBuffer);
    }

    void run(double secsLeft) {
        runWithSonar(secsLeft);
    }

    void runWithSonar(double secsLeft) {

        int columnIndex = 2;
        boolean red_ = !robot.blueTeam;
        boolean blue_ = robot.blueTeam;
        boolean short_ = robot.shortField;
        boolean long_ = !robot.shortField;

        // INCREASE SPEED ONLY IF NEEDED, HIGH SPEED MAKES IT LESS ROBUST
        {
            if (red_ && short_) speedIncrease = 2.2;
            if (red_ && long_) speedIncrease = 2.2;
            if (blue_ && short_) speedIncrease = 2.2;
            if (blue_ && long_) speedIncrease = 2.2;
        }
        // START TIMER
        {
            secsLeftAtStart = secsLeft;
            timer.reset();
            if (noTimeLeft(1)) return;
        }
        // DETERMINE WHAT COLUMN
        {
            if (vu.targetSeen()) {
                columnIndex = vu.lastTargetSeenNo;
            } else {
                columnIndex = 2; //go middle if no vuforia
            }
            showIfTargetSeen();
            if (noTimeLeft(1)) return;
        }
        // ROBOT SHOULD BE ON FLAT BACK AGAINST PLATFORM
        // IF THE HEADING IS OFF FIX THE HEADING AND BACK AAGAIN AGAINST THE PLATFORM
        if (Math.abs(robot.gyroDrift(0)) > 0.12) {

            int prevColor = robot.colorBeacon.getColorNumber();
            robot.colorBeacon.pink();
            robot.adjustTurnTo12();

            // if heading is off most likely is now too close to the glyph box
            // moving a bit to the left just in case
            robot.moveSideInches(-1,0.22,1);
            robot.colorBeacon.colorNumber(prevColor);

            // now back against the platform to reset the position
            if (red_ && short_) robot.moveInches(-1, 0.11, 2);
            if (red_ && long_) robot.moveInches(-1, 0.11, 2);
            if (blue_) robot.moveInches(1, 0.11, 2);
            if (noTimeLeft(1)) return;
        }
        // MOVE IN FRONT OF BOX USING ENCODERS
        {
            // robot.moveLinearGyroTrackingEnabled = true;
            robot.moveLinearGyroHeadingToTrack = 0;

            if (red_ && long_) robot.moveInches(2, 0.22 * speedIncrease, 4);
            if (blue_ && long_) robot.moveInches(-2, 0.22 * speedIncrease, 4);
            if (blue_ && long_) robot.turnTo6();
            showIfTargetSeen();

            double redShortEncoder[] = {24, 16.5, 10}; // L C R
            double redLongEncoder[] = {20.5, 14, 6.5}; // L C R
            double blueShortEncoder[] = {10, 16.5, 24}; // L C R
            double blueLongEncoder[] = {6.5, 14, 20.5}; // L C R
            double moveDistance = 8;

            if (red_ && short_) moveDistance = redShortEncoder[columnIndex - 1];
            if (red_ && long_) moveDistance = redLongEncoder[columnIndex - 1];
            if (blue_ && short_) moveDistance = blueShortEncoder[columnIndex - 1];
            if (blue_ && long_) moveDistance = blueLongEncoder[columnIndex - 1];

            if (red_ && short_) robot.moveInches(moveDistance, 0.44 * speedIncrease, 8);
            if (red_ && long_) robot.moveSideInches(moveDistance, 0.44 * speedIncrease, 8);
            if (blue_ && short_) robot.moveInches(-moveDistance, 0.44 * speedIncrease, 8);
            if (blue_ && long_) robot.moveSideInches(-moveDistance, 0.44 * speedIncrease, 8);
            showIfTargetSeen();

            if (noTimeLeft(1)) return;
        }
        // TURN TOWARDS THE BOX
        {
            if (red_ && short_) robot.turnTo3();
            if (red_ && long_) robot.adjustTurnTo12();
            if (blue_ && short_) robot.turnTo3();
            if (blue_ && long_) robot.adjustTurnTo6();
            showIfTargetSeen();

            if (noTimeLeft(1)) return;
        }
        // MOVE IN FRONT OF THE COLUMN USING SONAR
        {
            double redShortSonar[] = {0.74, 0.64, 0.55}; // L C R
            double redLongSonar[] = {0.43, 0.34, 0.25}; // L C R
            double blueShortSonar[] = {0.55, 0.64, 0.74}; // L C R
            double blueLongSonar[] = {0.25, 0.34, 0.43}; // L C R
            double columnSonarPos = 0;

            robot.sonarMaxAdjust = 0.12; // one column

            if (red_ && short_) columnSonarPos = redShortSonar[columnIndex - 1];
            if (red_ && long_) columnSonarPos = redLongSonar[columnIndex - 1];
            if (blue_ && short_) columnSonarPos = blueShortSonar[columnIndex - 1];
            if (blue_ && long_) columnSonarPos = blueLongSonar[columnIndex - 1];

            if (red_ && short_) robot.moveBySonarRight(columnSonarPos, 0.44, 3);
            if (red_ && long_) robot.moveBySonarRight(columnSonarPos, 0.44, 3);
            if (blue_ && short_) robot.moveBySonarLeft(columnSonarPos, 0.44, 3);
            if (blue_ && long_) robot.moveBySonarLeft(columnSonarPos, 0.44, 3);
            showIfTargetSeen();

            if (noTimeLeft(2)) return;
        }
        // TURN TOWARDS THE BOX
        {
            if (red_ && short_) robot.turnTo3();
            if (red_ && long_) robot.turnTo12();
            if (blue_ && short_) robot.turnTo3();
            if (blue_ && long_) robot.turnTo6();
            robot.showTeamColor();

            if (noTimeLeft(1)) return;
        }
        //PUT THE GLYPH IN
        {
            robot.moveLift(-1);
            robot.moveInches(4, 0.15, 3);
            robot.openClawZero();
            robot.showTeamColor();

            // IF TIME DO WIGGLE
            if (timeLeft(2)) {
                robot.closeClawRight();
                waitMillis(333);
                robot.closeClawLeft();
                waitMillis(333);
                robot.openClawZero();
            }
            robot.moveInches(-3, 0.33, 3);
            robot.showTeamColor();
            if (noTimeLeft(1)) return;

            robot.moveInches(3.5, 0.33, 3);
            robot.moveInches(-4, 0.33, 3);

        }
        // IF THERE IS TIME TURN AROUND
        if (!noTimeLeft(2)) {
            if (red_ && short_) robot.turnTo9();
            if (red_ && long_) robot.turnTo6();
            if (blue_ && short_) robot.turnTo9();
            if (blue_ && long_) robot.turnTo12();
        }

        robot.showTeamColor();
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

