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
    HardwareMap hwMap = null;
    double speedIncrease = 1.8;

    //********************************* END VARIABLE ********************************************//

    void init(Team_Hardware_V9 aRobot) {

        robot = aRobot;
    }

    boolean noTimeLeft(double secBuffer) {
        waitMillis(6);
        if ((secsLeftAtStart - timer.seconds()) > secBuffer) {
            return false;
        }
        robot.moveLift(-3);
        robot.openClawZero();
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

        robot.colorBeacon.displayStatus();

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
            if (robot.vu.targetSeen()) {
                columnIndex = robot.vu.lastTargetSeenNo;
            } else {
                columnIndex = 2; //go middle if no vuforia
            }
            showIfTargetSeen();
            if (noTimeLeft(1)) return;
        }
        // ROBOT SHOULD BE OFF THE PLATOFRM WITH THE BACK AGAINST PLATFORM
        {
            if (!(blue_ && long_)) {
                robot.adjustTurnTo12();
            }
            if (blue_ && short_) robot.moveSideInches(3, 0.22, 3);
        }
        // MOVE IN FRONT OF BOX USING ENCODERS
        {
            robot.moveLinearGyroTrackingEnabled = true;
            robot.moveLinearGyroHeadingToTrack = 0;

            if (red_ && long_) robot.moveInches(-2, 0.66, 4);


            if (blue_ && long_) {
                robot.moveInches(-4, 1, 2);
                robot.turnTo9();
                robot.turnTo6();
            } else {
                robot.adjustTurnTo12();
            }
            showIfTargetSeen();

            double redShortEncoder[] = {19, 13.5, 7}; // L C R
            double redLongEncoder[] = {5.5 + 12 + 8, 5.5 + 11, 5.5}; // L C R
            double blueShortEncoder[] = {8, 14.5, 20}; // L C R
            double blueLongEncoder[] = {13.5, 10.5 + 11, 10.5 + 11 + 6.5}; // L C R
            double moveDistance = 8;

            if (red_ && short_) moveDistance = redShortEncoder[columnIndex - 1];
            if (red_ && long_) moveDistance = redLongEncoder[columnIndex - 1];
            if (blue_ && short_) moveDistance = blueShortEncoder[columnIndex - 1];
            if (blue_ && long_) moveDistance = blueLongEncoder[columnIndex - 1];

            if (short_) {
                robot.moveLinearGyroTrackingEnabled = true;
                robot.moveLinearGyroHeadingToTrack = 0;
            }

            if (red_ && short_) robot.moveInches(moveDistance, 0.44 * speedIncrease, 11);
            if (red_ && long_) robot.moveSideInches(moveDistance, 0.44 * speedIncrease, 11);
            if (blue_ && short_) robot.moveInches(-moveDistance, 0.44 * speedIncrease, 11);
            if (blue_ && long_) robot.moveSideInches(-moveDistance, 0.44 * speedIncrease, 11);
            showIfTargetSeen();

            if (noTimeLeft(1)) return;
        }
        // TURN TOWARDS THE BOX
        {
            if (red_ && short_) robot.adjustTurnTo3();
            if (red_ && long_) robot.adjustTurnTo12();
            if (blue_ && short_) robot.adjustTurnTo3();
            if (blue_ && long_) robot.adjustTurnTo6();
            showIfTargetSeen();

            if (noTimeLeft(1)) return;
        }
        // MOVE IN FRONT OF THE COLUMN USING SONAR
        {
            double redShortSonar[] = {0.74, 0.64, 0.55}; // L C R
            double redLongSonar[] = {0.44, 0.34, 0.25}; // L C R
            double blueShortSonar[] = {0.55, 0.64, 0.74}; // L C R
            double blueLongSonar[] = {0.25, 0.34, 0.43}; // L C R
            double columnSonarPos = 0;

            robot.sonarMaxAdjust = 0.12; // one column

            if (red_ && short_) columnSonarPos = redShortSonar[columnIndex - 1];
            if (red_ && long_) columnSonarPos = redLongSonar[columnIndex - 1];
            if (blue_ && short_) columnSonarPos = blueShortSonar[columnIndex - 1];
            if (blue_ && long_) columnSonarPos = blueLongSonar[columnIndex - 1];

            if (red_ && short_) robot.moveBySonarRight(columnSonarPos, 0.33, 3);
            if (red_ && long_) robot.moveBySonarRight(columnSonarPos, 0.33, 3);
            if (blue_ && short_) robot.moveBySonarLeft(columnSonarPos, 0.33, 3);
            if (blue_ && long_) robot.moveBySonarLeft(columnSonarPos, 0.33, 3);
            showIfTargetSeen();

            if (noTimeLeft(2)) return;
        }
        // TURN TOWARDS THE BOX
        {
            if (red_ && short_) robot.turnTo3();
            if (red_ && long_) robot.turnTo12();
            if (blue_ && short_) robot.turnTo3();
            if (blue_ && long_) robot.turnTo6();

            if (noTimeLeft(2)) return;
        }
        //PUT THE GLYPH IN
        {
            robot.moveLift(-8);
            if (long_) robot.moveInches(2, 0.44, 2);
            robot.openClawWide();
            robot.moveInches(-2, 0.44, 2);
            if (long_) {
                if (red_) robot.moveInches(4, 0.33, 3);
                else robot.moveInches(4, 0.33, 3);
            } else {
                robot.moveInches(4, 0.33, 3);
            }

            // IF TIME TUCK IN
            if (timeLeft(2)) {
                robot.moveInches(-1.5, 0.66, 3);
                robot.moveSideInches(-3, 0.66, 3);
                robot.moveSideInches(6, 0.66, 3);
                robot.moveSideInches(-4, 0.66, 3);
                robot.moveInches(3, 0.22, 3);
                robot.openClawZero();
            }
            robot.moveInches(-4, 0.66, 3);
        }
        // IF THERE IS TIME TURN AROUND
        if (!noTimeLeft(2)) {
            robot.moveLift(2);
            if (red_ && short_) robot.turnTo9();
            if (blue_ && short_) robot.turnTo9();
            if (blue_ && long_) robot.turnTo12();
            if (red_ && long_) robot.turnTo6();
        }
        robot.stopRobot();
        robot.colorBeacon.displayStatus();
    }

    void waitVu(double millis) {

        for (int i = 0; i < (int) (millis / 10); i++) {
            if (robot.vu.targetSeen()) {
                robot.colorBeacon.modeVuforiaFound = true;
                robot.colorBeacon.displayStatus();
                return;
            }
            waitMillis(11);
        }
    }

    private void waitMillis(double millis) {

        sleep((long) millis);
    }

    void showIfTargetSeen() {
        if (robot.vu.targetSeen()) {
            robot.colorBeacon.modeVuforiaFound = true;
            robot.colorBeacon.displayStatus();
        } else {
            robot.colorBeacon.modeVuforiaFound = false;
            robot.colorBeacon.displayStatus();
        }
    }
}

