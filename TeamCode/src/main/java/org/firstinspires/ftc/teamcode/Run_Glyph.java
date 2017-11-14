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

    boolean timeLeft() {

        if ((secsLeftAtStart - timer.seconds()) > 1) {
            return true;
        }
        robot.stopRobot();
        return false;
    }

    void run(double secsLeft) {

        if (robot == null) {
            return;
        }

        secsLeftAtStart = secsLeft;
        timer.reset();
        if (!timeLeft()) return;

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
            if (!timeLeft()) return;
        }
        showIfTargetSeen();

        //****  2. MOVE OFF THE PLATFORM *********************************************************//

        if (robot.blueTeam) {
            robot.moveInches(10 * direction, 0.6, 0.66);
        } else {
            robot.moveInches(13.5 * direction, 0.8, 0.66);
        }
        showIfTargetSeen();
        if (!timeLeft()) return;

        //****  3. CORRECT HEADING IF NEEDED ********************** ******************************//

        robot.turnTo12();
        showIfTargetSeen();
        if (!timeLeft()) return;

        //****  4. DETERMINE WHAT COLUMN  *********************************//
        int index = 0;
        if (vu.targetSeen()) {
            index = vu.lastTargetSeenNo;
            showIfTargetSeen();
        } else {
            index = 2; //go middle if no vuforia
        }
        if (!timeLeft()) return;

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
            if (!timeLeft()) return;

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
        if (!timeLeft()) return;

        //****  6. PUT THE GLYPH IN  *************************************************************//
        robot.moveInches(5.5, 0.22, 1);
        robot.openClawAuto();
        waitMillis(22);
        if (!timeLeft()) return;

        //****  7. BACKOFF ***********************************************************************//
        robot.moveInches(-4, 1, 1);
        robot.setClawPosZero();
        robot.moveInches(2, 1, 1);
        robot.moveInches(-4, 1, 1);
        robot.stopRobot();

        //********************************* END LOOP *****************************************//
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

