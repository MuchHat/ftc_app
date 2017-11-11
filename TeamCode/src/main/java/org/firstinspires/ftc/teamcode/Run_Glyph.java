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

        double[] moveDistance = {15.4, 9, 3};
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

        if(robot.blueTeam) {
            robot.moveInches(-4, 0.4);
        }
        for (int i = 0; i < 10; i++) {
            if (vu.targetSeen()) {
                break;
            }
            robot.colorBeacon.white();
            waitMillis(100);
            if (!timeLeft()) return;
        }
        showIfTargetSeen();

        //****  2. MOVE OFF THE PLATFORM *********************************************************//

        if(robot.blueTeam){
            robot.moveInches(7 * direction, 0.8);
        }
        else {
            robot.moveInches(13.5 * direction, 1.0);
        }
        waitMillis(22);
        showIfTargetSeen();
        if (!timeLeft()) return;

        //****  3. CORRECT HEADING IF NEEDED ********************** ******************************//

        robot.turnTo12();
        showIfTargetSeen();
        waitMillis(22);
        if (!timeLeft()) return;

        //****  5. MOVE IN FRONT OF THE BOX L/M/R PER THE WUMARK *********************************//

        int index = 0;
        if (vu.targetSeen()) {
            index = vu.lastTargetSeenNo;
            showIfTargetSeen();
            //robot.beaconBlink(index + 1);
        } else {
            index = 2; //go middle if no vuforia
        }
        waitMillis(22);
        if (!timeLeft()) return;

        //****  SPECIAL STEPS FOR THE LONG FIELD *************************************************//

        if (!robot.shortField) {
            robot.moveInches(moveDistanceFirstLegLongField * direction, 1.0);
            waitMillis(22);

            robot.turnTo9();
            waitMillis(22);
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
        waitMillis(22);
        if (!timeLeft()) return;

        robot.stopRobot();
        waitMillis(22);
        if (!timeLeft()) return;

        //****  8. PUT THE GLYPH IN  *************************************************************//

        robot.moveInches(6.5, 1.0);
        if (!timeLeft()) return;

        robot.openClawAuto();
        waitMillis(22);
        if (!timeLeft()) return;

        //****  9. BACKOFF ***********************************************************************//

        robot.moveInches(-3.5, 1.0);
        robot.setClawPosZero();
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

