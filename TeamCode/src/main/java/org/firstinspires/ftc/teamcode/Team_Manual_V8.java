package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/* NOTES

https://github.com/ftctechnh/ftc_app/wiki/Identifying-Vuforia-VuMarks

Vuforia key

Ac6cr63/////AAAAGUsQTEyyG0kggwF13U8WoMlPgXZiUoKR9pf2nlfhVVfvDXFsTn0wufoywxzibq+y5BGBa2cChKWAcUkKaD9/ak5lwCm9Wp3Osk9omsMR0YYoxt4TuPktrflK4HuTH8cOAQA8YDuOs/SO/cgOmWbQZtRXN/lFkUwGZA9eiV5D8730BG2SBLPR4A9rcFs0Fp/yPgcm4Zsh5Kv2Ct8XjJXmXk5mAjERZ5B6hKQzf/4wd9tSQ6BeQLvsgd5nI0Pj+K1NHI4EyHdFyxCPu91AMcCsXCLjkABfYt11Zhxu1uYaFF/AcN3eBHRwprVpDEBBXOMnD4BRCj0xxYYPWWO6g4gcjqBPgBos5nCDk43KipEeX22z

 */

//********************************* MAIN OP CLASS ************************************************//

@TeleOp(name = "Manual V7", group = "Team")
// @Disabled
public class Team_Manual_V8 extends LinearOpMode {

    //********************************* HW VARIABLES *********************************************//
    private Team_Hardware_V3 robot = new Team_Hardware_V3();

    //********************************* MOVE STATES **********************************************//
    private ElapsedTime totalRuntime = null;

    //********************************* CONSTANTS ************************************************//
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

        robot.modernRoboticsI2cGyro.calibrate();
        // Wait until the gyro calibration is complete
        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating gyro", "... do NOT move");
            telemetry.addData("calibrating gyro", "%s", Math.round(controlRuntime.seconds()));
            boolean blink = Math.round(controlRuntime.seconds() * 2) % 2 == 0;
            if (blink) {
                if (blueTeam) robot.colorBeacon.blue();
                if (!blueTeam) robot.colorBeacon.red();
            }
            if (!blink) {
                robot.colorBeacon.off();
            }
            loaded = true;
            telemetry.update();
            sleep(66);
        }
        if (blueTeam) robot.colorBeacon.blue();
        if (!blueTeam) robot.colorBeacon.red();

        telemetry.clear();
        telemetry.update();
        telemetry.addData("driver", "CLICK  >>> to START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && loaded) {

            //********************************* CONTROL LOOP *************************************//

            double crrLoopTime = loopRuntime.nanoseconds() / 1000000; // covert to millis
            loopRuntime.reset();

            updateTelemetry();

            //********************************* MANUAL MODE **************************************//
            //              BOTH GAME PADS ARE MAPPED THE SAME AT THIS TIME                       //
            //            IF THE SAME COMMAND IS GIVEN ON BOTH GAME PAD 1 WINS                    //
            //********************************* MANUAL MODE **************************************//

            // ***************************** control: DRIVES  ********************************//
            {
                double xInput = 0;
                double yInput = 0;

                if (Math.abs(gamepad2.left_stick_y) > 0.06) {
                    yInput = -gamepad2.left_stick_y;
                }
                if (Math.abs(gamepad2.left_stick_x) > 0.06) {
                    xInput = gamepad2.left_stick_x;
                }

                if (Math.abs(gamepad1.left_stick_y) > 0.06) {
                    yInput = -gamepad1.left_stick_y;
                }
                if (Math.abs(gamepad1.left_stick_x) > 0.06) {
                    xInput = gamepad1.left_stick_x;
                }

                robot.leftDriveControl = yInput * robot.driveDefaultSpeed;
                robot.rightDriveControl = yInput * robot.driveDefaultSpeed;

                robot.leftDriveControl += xInput * robot.turnDefaultSpeed;
                robot.rightDriveControl -= xInput * robot.turnDefaultSpeed;

                robot.leftDriveControlBack = robot.leftDriveControl;
                robot.rightDriveControlBack = robot.rightDriveControl;

                robot.setDrives();

                if (xInput != 0) robot.headingControl = robot.modernRoboticsI2cGyro.getHeading();
            }

            // ********************************  control: LIFT  ******************************//
            {
                double liftInput = 0;

                if (Math.abs(gamepad2.right_stick_y) > 0.06) {
                    liftInput = gamepad2.right_stick_y;
                }

                if (Math.abs(gamepad1.right_stick_y) > 0.06) {
                    liftInput = gamepad1.right_stick_y;
                }

                double liftDefaultSpeed = 1.5;
                robot.liftControl = liftInput * liftDefaultSpeed;
                robot.setDrives();
            }

            // ********************************  control: TURNS 90  **************************//
            if (gamepad1.dpad_right ||
                    gamepad2.dpad_right) {
                robot.turn(90);
            }

            // ********************************  control: TURNS -90
            if (gamepad1.dpad_left ||
                    gamepad2.dpad_left) {
                robot.turn(-90);
            }

            // ********************************  control: TURN FACING THE CRYPTO BOX  ********//
            if (gamepad1.dpad_up ||
                    gamepad2.dpad_up) {
                robot.turnToHeading(0);
            }

            // ********************************  control: TURNS 180  *************************//
            if (gamepad1.dpad_down ||
                    gamepad2.dpad_down) {
                robot.turn(180);
            }

            // ********************************  control: SMALL STEP FORWARD  ****************//
            if (gamepad1.y ||
                    gamepad2.y) {
                double step = 11;

                robot.move(step);
            }

            // ********************************  control: SMALL STEP REVERSE  ****************//
            if (gamepad1.a ||
                    gamepad2.a) {
                double step = 11;

                robot.move(-step);
            }

            // ********************************  control: SMALL STEP LEFT  *******************//
            if (gamepad1.x ||
                    gamepad2.x) {
                double step = 11;

                robot.moveSide(step);
            }

            // ********************************  control: SMALL STEP RIGHT  ******************//
            if (gamepad1.b ||
                    gamepad2.b) {
                double step = -11;

                robot.moveSide(step);
            }

            // ********************************  control: CLAW OPEN  *************************//
            if (gamepad2.left_trigger != 0) {
                robot.leftClawControl -= gamepad2.left_trigger * robot.servoDefaultSpeed * crrLoopTime;
                robot.rightClawControl += gamepad2.left_trigger * robot.servoDefaultSpeed * crrLoopTime;
                robot.setServos();
            }
            if (gamepad1.left_trigger != 0) {
                robot.leftClawControl -= gamepad1.left_trigger * robot.servoDefaultSpeed * crrLoopTime;
                robot.rightClawControl += gamepad1.left_trigger * robot.servoDefaultSpeed * crrLoopTime;
                robot.setServos();
            }

            // ********************************  control: CLAW CLOSE  ************************//
            if (gamepad2.right_trigger != 0) {
                robot.leftClawControl += gamepad2.right_trigger * robot.servoDefaultSpeed * crrLoopTime;
                robot.rightClawControl -= gamepad2.right_trigger * robot.servoDefaultSpeed * crrLoopTime;
                robot.setServos();
            }
            if (gamepad1.right_trigger != 0) {
                robot.leftClawControl += gamepad1.right_trigger * robot.servoDefaultSpeed * crrLoopTime;
                robot.rightClawControl -= gamepad1.right_trigger * robot.servoDefaultSpeed * crrLoopTime;
                robot.setServos();
            }
            // ********************************  control: CLAW PREDEF OPEN  ******************//
            if (gamepad1.left_bumper ||
                    gamepad2.left_bumper) {
                robot.openClaw();
            }
            // ********************************  control: CLAW PREDEF CLOSE  *****************//
            if (gamepad1.right_bumper ||
                    gamepad2.right_bumper) {
                robot.closeClaw();
            }

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

        telemetry.update();
    }

    // ************************** OP END *********************************************************//
}

