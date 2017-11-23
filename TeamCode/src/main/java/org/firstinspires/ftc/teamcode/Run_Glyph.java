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
    double speedIncrease = 1.0;

    //********************************* END VARIABLE ********************************************//

      void init(Team_Hardware_V9 aRobot, HardwareMap aHwMap) {
        hwMap = aHwMap;
        robot = aRobot;
        vu.init(hwMap);
    }

    boolean noTimeLeft() {
        waitMillis(11);
        if ((secsLeftAtStart - timer.seconds()) > 1) {
            return false;
        }
        // robot.stopRobot();
        // return true;
        return false;
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

        // INCREASE SPPED ONLY IF NEEDED, HIGH SPEED MAKES IT LESS ROBUST
        {
            if (red_ && short_) speedIncrease = 1.0;
            if (red_ && long_) speedIncrease = 1.0;
            if (blue_ && short_) speedIncrease = 1.0;
            if (blue_ && long_) speedIncrease = 1.0;
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
            if (red_ && short_) robot.moveInches(6.7, 0.66 * speedIncrease, 4);
            if (red_ && long_) robot.moveInches(6.7, 0.66 * speedIncrease, 4);
            if (blue_ && short_) robot.moveInches(-6.7, 0.66 * speedIncrease, 4);
            if (blue_ && long_) robot.moveInches(-6.7, 0.66 * speedIncrease, 4);

            if (noTimeLeft()) return;
        }
        // BACK AGAINST THE PLATFORM
        {
            if (red_) robot.moveInches(-2, 0.11 * speedIncrease, 2);
            if (red_ && long_) robot.moveInches(-3, 0.11 * speedIncrease, 2);
            if (blue_) robot.moveInches(2, 0.11 * speedIncrease, 2);
            if (noTimeLeft()) return;
        }
        // MOVE IN FRONT OF BOX USING ENCODERS
        {
            if (red_ && long_) robot.moveInches(2, 0.88 * speedIncrease, 4);
            if (blue_ && long_) robot.moveInches(-4, 0.88 * speedIncrease, 4);

            if (blue_ && long_) robot.turnTo6();

            double redShortEncoder[] = {24, 16.5, 10}; // L C R
            double redLongEncoder[] = {20.5, 14, 6.5}; // L C R
            double blueShortEncoder[] = {10, 16.5, 24}; // L C R
            double blueLongEncoder[] = {6.5, 14, 20.5}; // L C R
            double moveDistance = 8;

            if (red_ && short_) moveDistance = redShortEncoder[columnIndex - 1];
            if (red_ && long_) moveDistance = redLongEncoder[columnIndex - 1];
            if (blue_ && short_) moveDistance = blueShortEncoder[columnIndex - 1];
            if (blue_ && long_) moveDistance = blueLongEncoder[columnIndex - 1];

            if (red_ && short_) robot.moveInches(moveDistance, 0.44 * speedIncrease, 6);
            if (red_ && long_) robot.moveSideInches(moveDistance, 0.44 * speedIncrease, 6);
            if (blue_ && short_) robot.moveInches(-moveDistance, 0.44 * speedIncrease, 6);
            if (blue_ && long_) robot.moveSideInches(-moveDistance, 0.44 * speedIncrease, 6);
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
            double redShortSonar[] = {0.75, 0.65, 0.56}; // L C R
            double redLongSonar[] = {0.43, 0.34, 0.25}; // L C R
            double blueShortSonar[] = {0.56, 0.65, 0.75}; // L C R
            double blueLongSonar[] = {0.26, 0.34, 0.43}; // L C R
            double columnSonarPos = 0;

            robot.sonarMaxAdjust = 0.10; // half a column

            if (red_ && short_) columnSonarPos = redShortSonar[columnIndex - 1];
            if (red_ && long_) columnSonarPos = redLongSonar[columnIndex - 1];
            if (blue_ && short_) columnSonarPos = blueShortSonar[columnIndex - 1];
            if (blue_ && long_) columnSonarPos = blueLongSonar[columnIndex - 1];

            if (red_ && short_) robot.moveBySonarRight(columnSonarPos, 0.44 * speedIncrease, 1);
            if (red_ && long_) robot.moveBySonarRight(columnSonarPos, 0.44 * speedIncrease, 1);
            if (blue_ && short_) robot.moveBySonarLeft(columnSonarPos, 0.44 * speedIncrease, 1);
            if (blue_ && long_) robot.moveBySonarLeft(columnSonarPos, 0.44 * speedIncrease, 1);

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
            robot.moveInches(5.5, 0.15 * speedIncrease, 1);
            robot.openClawAuto();
            waitMillis(22);
            if (noTimeLeft()) return;
        }

        //BACKOFF
        {
            robot.moveInches(-6, 0.44 * speedIncrease, 1);
            robot.setClawPosZero();
            if (noTimeLeft()) return;
            robot.moveInches(4, 0.44 * speedIncrease, 1);
            robot.moveInches(-4, 0.44 * speedIncrease, 1);
            robot.stopRobot();
        }

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

