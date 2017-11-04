package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

//********************************* MAIN OP CLASS ************************************************//

public class Run_Glyph {

    //********************************* HW VARIABLES *********************************************//
    Team_Hardware_V9 robot = null;
    double secsLeftAtStart = 30;
    ElapsedTime timer = new ElapsedTime();

    //********************************* CONSTANTS ************************************************//
    private Vu vu = new Vu();
    private HardwareMap hwMap = null;

    void init(Team_Hardware_V9 aRobot, HardwareMap aHwMap) {

        hwMap = aHwMap;
        robot = aRobot;
        vu.init(hwMap);
    }

    boolean timeLeft() {

        return true;//(secsLeftAtStart - timer.seconds()) > 2;
    }

    void run(double secsLeft) {

        if (robot == null) {
            return;
        }

        secsLeftAtStart = secsLeft;
        timer.reset();
        if (!timeLeft()) return;

        int[] moveDistance = {580, 358, 161};
        int[] moveDistanceLongField = {419, 197, 0};

        int[] vuforiaValues = {460, 415, 340};
        int[] vuforiaValuesLongField = {296, 355, 347};

        int moveDistanceFirstLegLongField = 161;

        //****  0. ADJUST VARIABLES DEPENDING ON THE FIELD **************************************//

        double direction = 1.0;
        if (robot.blueTeam) direction = -1.0;

        if (!robot.shortField) {
            moveDistance[0] = moveDistanceLongField[0];
            moveDistance[1] = moveDistanceLongField[1];
            moveDistance[2] = moveDistanceLongField[2];
        }
        if (!robot.shortField) {
            vuforiaValues[0] = vuforiaValuesLongField[0];
            vuforiaValues[1] = vuforiaValuesLongField[1];
            vuforiaValues[2] = vuforiaValuesLongField[2];
        }

        //****  1. WAIT IF NEEDED FOR VUFORIA TO LOCK ON THE TARGET ******************************//

        for (int i = 0; i < 10; i++) {
            robot.colorBeacon.teal();
            showGreenIfTargetSeen();
            if (vu.targetSeen()) {
                break;
            }
            waitMillis(100);
            if (!timeLeft()) return;
        }

        robot.colorBeacon.white();
        //****  2. MOVE OFF THE PLATFORM *********************************************************//

        robot.moveInches(432 / 24.5 * direction, 0.88);
        waitMillis(222);
        showGreenIfTargetSeen();
        if (!timeLeft()) return;

        //****  3. CORRECT HEADING IF NEEDED ********************** ******************************//

        if (!robot.blueTeam) {
            robot.turnTo12();
        } else {
            robot.turnTo6();
        }
        showGreenIfTargetSeen();
        waitMillis(222);
        if (!timeLeft()) return;

        //****  4. BACK AGAINST THE PLATFORM TO START FROM A KNOWN POS ***************************//

        if(robot.blueTeam)
            robot.moveInches(-5, 0.88);
        else
            robot.moveInches(-260 / 24.5, 0.88);
        showGreenIfTargetSeen();
        waitMillis(222);
        if (!timeLeft()) return;

        //****  5. MOVE IN FRONT OF THE BOX L/M/R PER THE WUMARK *********************************//

        int index = 0;
        if (vu.targetSeen()) {
            index = vu.lastTargetSeenNo;
        } else {
            index = 2; //go midedle if no vuforia
        }
        showGreenIfTargetSeen();
        robot.beaconBlink(index + 1);
        waitMillis(222);
        if (!timeLeft()) return;

        //****  SPECIAL STEPS FOR THE LONG FIELD *************************************************//

        if (!robot.shortField) {
            robot.moveInches(moveDistanceFirstLegLongField / 24.5, 0.88);
            waitMillis(222);

            robot.turnTo9();
            waitMillis(222);
        }
        if (!timeLeft()) return;

        //****  CONTINUES THE SAME WITH THE SHORT FIELD ******************************************//

        if (index != 0) {
            robot.moveInches(moveDistance[index - 1] / 24.5, 0.88);
            waitMillis(222);
        }
        if (!timeLeft()) return;

        //****  6. FINE ADJUST THE POS USING VUMARK AS AN ANCHOR *********************************//

        if (vu.targetSeen()) {

            int errDis = 15; // 1 left 2 center 3 right
            index = vu.getLastTargetSeenNo() - 1;
            int desiredX = vuforiaValues[index];

            waitMillis(222);
            robot.moveInches(50 / 24.5, 0.33);

            double attempts = 0;

            if (vu.getX() - desiredX > errDis) {
                while (vu.getX() - desiredX > errDis && attempts < 11) {
                    robot.colorBeacon.purple();
                    robot.moveInches(-10 / 24.5, 0.88);
                    attempts++;
                    if (!timeLeft()) return;
                }
            } else if (vu.getX() - desiredX < -errDis) {
                while (vu.getX() - desiredX < -errDis && attempts < 11) {
                    robot.colorBeacon.purple();
                    robot.moveInches(10 / 24.5, 0.88);
                    attempts++;
                    if (!timeLeft()) return;
                }
            }

        }
        robot.showTeamColor();

        //****  7. TURN 90 TOWARDS THE BOX ******************************************************//

        if (robot.shortField) {
            robot.turnTo3();
        } else {
            if (robot.blueTeam) {
                robot.turnTo6();
            } else {
                robot.turnTo12();
            }
        }
        waitMillis(222);
        if (!timeLeft()) return;

        robot.stopRobot();
        waitMillis(222);

        //****  8. PUT THE GLYPH IN  *************************************************************//

        robot.moveInches(6.5, 0.88);

        robot.rightClaw.setPosition(0.75);
        robot.leftClaw.setPosition(0.22);
        waitMillis(111);
        if (!timeLeft()) return;

        //****  9. BACKOFF ***********************************************************************//

        robot.moveInches(-4, 0.88);

        //****  10. TURN 180 AND TUCK IT IN ******************************************************//

        if(robot.shortField)
        {
            robot.turnTo9();
            if (!timeLeft()) return;
            robot.moveInches(-3, 0.88);
            if (!timeLeft()) return;
        }
        else
            robot.turnTo12();

        //****  11. MOVE 1 INCH AWAY FROM BOX FOR THE REST POSITION ******************************//

        robot.moveInches(1, 0.88);
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

    void showGreenIfTargetSeen() {
        if (vu.targetSeen()) {
            robot.colorBeacon.green();
        } else {
            robot.colorBeacon.yellow();
        }
    }
}

