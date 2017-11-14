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

        double moveDistanceFirstLegLongField = 2;

        //****  0. ADJUST VARIABLES DEPENDING ON THE FIELD **************************************//

        double direction = 1.0;
        if (robot.blueTeam) direction = -1.0;

        if (!robot.shortField) {
            for(int i = 0; i<moveDistance.length;i++)
            {
                moveDistance[i] = moveDistanceLongField[i];
            }
        }

        //****  1. WAIT IF NEEDED FOR VUFORIA TO LOCK ON THE TARGET ******************************//

        if(robot.blueTeam) {
            robot.moveInches(-6, 0.4);
        }
        for (int i = 0; i < 15; i++) {
            if (vu.targetSeen()) {
                break;
            }
            robot.colorBeacon.white();
            waitMillis(50);
            if (!timeLeft()) return;
        }
        showIfTargetSeen();

        //****  2. MOVE OFF THE PLATFORM *********************************************************//

        if(robot.blueTeam){
            robot.moveInches(10 * direction, 0.6);
        }
        else {
            robot.moveInches(13.5 * direction, 0.8);
        }
        showIfTargetSeen();
        if (!timeLeft()) return;

        //****  3. CORRECT HEADING IF NEEDED ********************** ******************************//

        robot.turnTo12();
        showIfTargetSeen();
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

        if(!robot.shortField)
        {
            robot.moveSide(moveDistanceLongField[index-1]*direction*25.4);
        }
        else {
            if (!timeLeft()) return;
            //****  SPECIAL STEPS FOR THE LONG FIELD *************************************************//
            if (!robot.shortField) {
                robot.moveInches(moveDistanceFirstLegLongField * direction, 1.0);
                waitMillis(10);

                robot.turnTo9();
                waitMillis(10);
            }
            if (!timeLeft()) return;

            //****  CONTINUES THE SAME WITH THE SHORT FIELD ******************************************//

            if (index != 0) {
                robot.moveInches(moveDistance[index - 1] * direction, 1.0);
                waitMillis(33);
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
        }
        //****  8. PUT THE GLYPH IN  *************************************************************//

        if (!timeLeft()) return;

        robot.openClawAuto();
        robot.moveInches(5.5, 1.0);
        waitMillis(22);
        if (!timeLeft()) return;

        //****  9. BACKOFF ***********************************************************************//

        robot.moveInches(-6, 1.0);
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

