package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/* NOTES

https://github.com/ftctechnh/ftc_app/wiki/Identifying-Vuforia-VuMarks

Vuforia key

Ac6cr63/////AAAAGUsQTEyyG0kggwF13U8WoMlPgXZiUoKR9pf2nlfhVVfvDXFsTn0wufoywxzibq+y5BGBa2cChKWAcUkKaD9/ak5lwCm9Wp3Osk9omsMR0YYoxt4TuPktrflK4HuTH8cOAQA8YDuOs/SO/cgOmWbQZtRXN/lFkUwGZA9eiV5D8730BG2SBLPR4A9rcFs0Fp/yPgcm4Zsh5Kv2Ct8XjJXmXk5mAjERZ5B6hKQzf/4wd9tSQ6BeQLvsgd5nI0Pj+K1NHI4EyHdFyxCPu91AMcCsXCLjkABfYt11Zhxu1uYaFF/AcN3eBHRwprVpDEBBXOMnD4BRCj0xxYYPWWO6g4gcjqBPgBos5nCDk43KipEeX22z

 */

//********************************* MAIN OP CLASS ************************************************//

@TeleOp(name = "Safe Auto V9", group = "Team")
// @Disabled
public class Safe_Auto_V9 extends LinearOpMode {

    //********************************* HW VARIABLES *********************************************//
    private Team_Hardware_V9 robot = new Team_Hardware_V9();

    //********************************* MOVE STATES **********************************************//
    private ElapsedTime totalRuntime = null;

    private Boolean blueTeam = true;
    private Boolean rightField = true;
    private Boolean loaded = false;

    // ************************** MAIN LOOP ******************************************************//
    @Override
    public void runOpMode() {

        //********************************* MAIN LOOP INIT ***************************************//
        robot.init(hardwareMap);

        ElapsedTime controlRuntime = new ElapsedTime();
        ElapsedTime loopRuntime = new ElapsedTime();
        totalRuntime = new ElapsedTime();

        controlRuntime.reset();
        if (blueTeam) robot.colorBeacon.blue();
        if (!blueTeam) robot.colorBeacon.red();

        robot.moveArm(robot.armPosZero[0], robot.armPosZero[1]);

        robot.modernRoboticsI2cGyro.calibrate();
        // Wait until the gyro calibration is complete
        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating gyro", "... do NOT move");
            telemetry.addData("calibrating gyro", "%s", Math.round(controlRuntime.seconds()));
            loaded = true;
            telemetry.update();
            sleep(66);
        }

        telemetry.clear();
        telemetry.update();
        telemetry.addData("driver", "CLICK  >>> to START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && loaded) {


            double crrLoopTime = loopRuntime.nanoseconds() / 1000000; // covert to millis
            loopRuntime.reset();
            updateTelemetry();

            //********************************* AUTO MOVE TO THE SAFE ZONE ***********************//

            robot.colorBeacon.yellow();

            // load vuforia, turn green if marker found
            waitMillis(333);
            robot.colorBeacon.green();

            // turn yellow if not found
            waitMillis(333);
            robot.colorBeacon.yellow();

            // drive based on distances from vuforia
            double stepsMove[]= {0, 0,  575,0, 0,  0,  15,  0};
            double stepsSide[]= {0, 0,  0,  0,  0,  0,  0,  0};
            double stepsTurns[]={0, 0,  0,  0,  85, 0,  0,  0};
            double stepsLift[]= {0, 0,  0,  0,  0,  0,  0,  0};
            double clawLeft[]=  {0, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0};
            double clawRight[]= {1, 0.55, 0.55, 0.55, 0.55, 0.55, 0.55, 1};

            waitMillis(333);
            for (int i = 0; i < stepsMove.length; i++) {

                robot.colorBeacon.teal();

                robot.leftClaw.setPosition(clawLeft[i]);
                robot.rightClaw.setPosition(clawRight[i]);

                if (stepsMove[i] != 0) {
                    robot.move(stepsMove[i]);
                }
                /*if (stepsSide[i] != 0) {
                    robot.moveSide(stepsSide[i]);
                }*/
                if (stepsTurns[i] != 0) {
                    robot.turn((int) stepsTurns[i]);
                }
                if (stepsLift[i] != 0) {
                    robot.moveLift(stepsLift[i]);
                }
                robot.colorBeacon.off();
                waitMillis(222);
            }

            robot.colorBeacon.green();
            robot.stopRobot();

            stop(); //stop the opMode

            //********************************* END LOOP *****************************************//
        }

    }

    private void waitMillis(double millis) {

        sleep((long) millis);
//        millis = Range.clip(millis, 0.01, millis);
//        runtimeWait.reset();
//        while (runtimeWait.nanoseconds() < millis * 1000 * 1000) {
//            idle();
//        }

    }

    private boolean stopTime(double totalSeconds) {
        return totalRuntime.seconds() > totalSeconds;
    }

    // ************************** TELEMETRY HELPER FUNCTIONS *************************************//

    private void updateTelemetry() {

        String field = rightField ? "right" : "left";
        String team = blueTeam ? "blue" : "red";

        telemetry.addData("left drive", "%.0f%%", robot.leftDriveControl * 100);
        telemetry.addData("right drive", "%.0f%%", robot.rightDriveControl * 100);
        telemetry.addData("lift", "%.0f%%", robot.liftControl * 100);
        telemetry.addData("left claw", "%.0f%%", robot.leftClawControl * 100);
        telemetry.addData("right claw", "%.0f%%", robot.rightClawControl * 100);

        telemetry.addData("crr heading", "%.2fdeg", (double) robot.modernRoboticsI2cGyro.getHeading());
        telemetry.addData("set heading", "%.2fdeg", (double) robot.headingControl);
        telemetry.addData("start heading", "%.2fdeg", robot.gameStartHeading);
        telemetry.addData("z angle", "%.2fdeg",
                (double) robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("team", team);
        telemetry.addData("field", field);
        //telemetry.addData("total runtime", "%.0fs", totalRuntime.seconds());

        telemetry.update();
    }

    // ************************** OP END *********************************************************//
}
