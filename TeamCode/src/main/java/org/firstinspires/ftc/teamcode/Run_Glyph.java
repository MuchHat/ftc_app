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

        return (secsLeftAtStart - timer.seconds()) > 2;
    }

    void run(double secsLeft) {

        if (robot == null) {
            return;
        }

        secsLeftAtStart = secsLeft;
        timer.reset();
        if (!timeLeft()) return;

        double[] moveDistance = {23.5, 14.5, 7.5};
        double[] moveDistanceLongField = {14, 7, 0};

        double moveDistanceFirstLegLongField = 6.5;

        //****  0. ADJUST VARIABLES DEPENDING ON THE FIELD **************************************//

        double direction = 1.0;
        if (robot.blueTeam) direction = -1.0;

        if (!robot.shortField) {
            moveDistance[0] = moveDistanceLongField[0];
            moveDistance[1] = moveDistanceLongField[1];
            moveDistance[2] = moveDistanceLongField[2];
        }

        //****  1. WAIT IF NEEDED FOR VUFORIA TO LOCK ON THE TARGET ******************************//

        showGreenIfTargetSeen();
        for (int i = 0; i < 10; i++) {
            if (vu.targetSeen()) {
                break;
            }
            robot.colorBeacon.purple();
            waitMillis(166);
            if (!timeLeft()) return;
        }
        showGreenIfTargetSeen();

        //****  2. MOVE OFF THE PLATFORM *********************************************************//

        robot.moveInches(18.5 * direction, 1.0);
        waitMillis(66);
        showGreenIfTargetSeen();
        if (!timeLeft()) return;

        //****  3. CORRECT HEADING IF NEEDED ********************** ******************************//

        robot.turnTo12();

        showGreenIfTargetSeen();
        waitMillis(66);
        if (!timeLeft()) return;

        //****  5. MOVE IN FRONT OF THE BOX L/M/R PER THE WUMARK *********************************//

        int index = 0;
        if (vu.targetSeen()) {
            index = vu.lastTargetSeenNo;
            showGreenIfTargetSeen();
            robot.beaconBlink(index + 1);
        } else {
            index = 2; //go midedle if no vuforia
        }
        waitMillis(66);
        if (!timeLeft()) return;

        //****  SPECIAL STEPS FOR THE LONG FIELD *************************************************//

        if (!robot.shortField) {
            robot.moveInches(moveDistanceFirstLegLongField * direction, 1.0);
            waitMillis(66);

            robot.turnTo9();
            waitMillis(66);
        }
        if (!timeLeft()) return;

        //****  CONTINUES THE SAME WITH THE SHORT FIELD ******************************************//

        if (index != 0) {
            robot.moveInches(moveDistance[index - 1] * direction, 1.0);
            waitMillis(66);
        }
        if (!timeLeft()) return;

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
        waitMillis(66);
        if (!timeLeft()) return;

        robot.stopRobot();
        waitMillis(66);

        //****  8. PUT THE GLYPH IN  *************************************************************//

        robot.moveInches(6.5, 1.0);

        robot.rightClaw.setPosition(0.75);
        robot.leftClaw.setPosition(0.22);
        waitMillis(66);
        if (!timeLeft()) return;

        //****  9. BACKOFF ***********************************************************************//

        robot.moveInches(-1, 1.0);

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

