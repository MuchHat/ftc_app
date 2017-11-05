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

@TeleOp(name = "Manual Blue V9", group = "Manual")
// @Disabled
public class Manual_Blue_V9 extends LinearOpMode {

    //********************************* HW VARIABLES *********************************************//
    private Team_Hardware_V9 robot = new Team_Hardware_V9();

    //********************************* MOVE STATES **********************************************//
    private ElapsedTime totalRuntime = null;

    //********************************* CONSTANTS ************************************************//

    // ************************** MAIN LOOP ******************************************************//

    @Override
    public void runOpMode() {

        //********************************* MAIN LOOP INIT ***************************************//
        robot.init(hardwareMap);

        robot.rightClaw.setPosition(0.17);
        robot.leftClaw.setPosition(0.84);

        ElapsedTime controlRuntime = new ElapsedTime();
        ElapsedTime loopRuntime = new ElapsedTime();
        totalRuntime = new ElapsedTime();

        controlRuntime.reset();
        robot.blueTeam = true;
        robot.showTeamColor();

        telemetry.clear();
        telemetry.update();
        telemetry.addData("driver", "CLICK  >>> to START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //********************************* CONTROL LOOP *************************************//

            double crrLoopTime = loopRuntime.nanoseconds() / 1000000; // covert to millis
            loopRuntime.reset();

            updateTelemetry();

            //********************************* MANUAL MODE **************************************//

            // ***************************** control: SIDE  ********************************//*/
            if (Math.abs(gamepad1.right_stick_x) > 0.11) {
                double sideInput = 0;

                if (Math.abs(gamepad1.right_stick_x) > 0.11) {
                    sideInput = gamepad1.right_stick_x;
                }

                robot.leftPowerControl = -sideInput * robot.driveDefaultSpeed;
                robot.rightPowerControl = sideInput * robot.driveDefaultSpeed;

                robot.leftPowerControlBack = sideInput * robot.driveDefaultSpeed;
                robot.rightPowerControlBack = -sideInput * robot.driveDefaultSpeed;

                robot.setDrivesByPower();
            }
            // ***************************** control: DRIVES  ********************************//
            else {
                double xInput = 0;
                double yInput = 0;

                robot.leftDistanceControl = 0;
                robot.rightDistanceControl = 0;
                robot.leftDistanceControlBack = 0;
                robot.rightDistanceControlBack = 0;

                if (Math.abs(gamepad1.left_stick_y) > 0.06) {
                    yInput = -gamepad1.left_stick_y;
                }
                if (Math.abs(gamepad1.left_stick_x) > 0.06) {
                    xInput = gamepad1.left_stick_x;
                }
                robot.leftPowerControl = yInput * robot.driveDefaultSpeed;
                robot.rightPowerControl = yInput * robot.driveDefaultSpeed;

                robot.leftPowerControl += xInput * robot.turnDefaultSpeed;
                robot.rightPowerControl -= xInput * robot.turnDefaultSpeed;

                robot.leftPowerControlBack = robot.leftPowerControl;
                robot.rightPowerControlBack = robot.rightPowerControl;

                robot.setDrivesByPower();

            }

            // ********************************  control: LIFT  ******************************//
            {
                double liftInput = 0;

                if (Math.abs(gamepad2.right_stick_y) > 0.11) {
                    liftInput = -gamepad2.right_stick_y;
                }

                if (Math.abs(gamepad1.right_stick_y) > 0.11) {
                    liftInput = -gamepad1.right_stick_y;
                }

                double liftDefaultSpeed = 1.5;
                robot.liftControl = liftInput * liftDefaultSpeed;
                robot.setDrivesByPower();
            }

            // ********************************  control:  ***********************************//
            if (gamepad1.dpad_right) {
                double sideInput = 0.3;
                double turnInput = 0.15;

                robot.leftDistanceControl = 0;
                robot.rightDistanceControl = 0;
                robot.leftDistanceControlBack = 0;
                robot.rightDistanceControlBack = 0;

                robot.leftPowerControl = -sideInput * robot.driveDefaultSpeed;
                robot.rightPowerControl = sideInput * robot.driveDefaultSpeed;
                robot.leftPowerControlBack = sideInput * robot.driveDefaultSpeed;
                robot.rightPowerControlBack = -sideInput * robot.driveDefaultSpeed;

                robot.leftPowerControl = turnInput * robot.driveDefaultSpeed;
                robot.rightPowerControl = -turnInput * robot.driveDefaultSpeed;
                robot.leftPowerControlBack = turnInput * robot.driveDefaultSpeed;
                robot.rightPowerControlBack = -turnInput * robot.driveDefaultSpeed;

                robot.setDrivesByPower();
                waitMillis(22);

                robot.leftPowerControl = 0;
                robot.rightPowerControl = 0;
                robot.leftPowerControlBack = 0;
                robot.rightPowerControlBack = 0;
                robot.setDrivesByPower();
            }

            // ********************************  control:   ***********************************//
            if (gamepad1.dpad_left) {
                double sideInput = -0.3;
                double turnInput = -0.15;

                robot.leftDistanceControl = 0;
                robot.rightDistanceControl = 0;
                robot.leftDistanceControlBack = 0;
                robot.rightDistanceControlBack = 0;

                robot.leftPowerControl = -sideInput * robot.driveDefaultSpeed;
                robot.rightPowerControl = sideInput * robot.driveDefaultSpeed;
                robot.leftPowerControlBack = sideInput * robot.driveDefaultSpeed;
                robot.rightPowerControlBack = -sideInput * robot.driveDefaultSpeed;

                robot.leftPowerControl = turnInput * robot.driveDefaultSpeed;
                robot.rightPowerControl = -turnInput * robot.driveDefaultSpeed;
                robot.leftPowerControlBack = turnInput * robot.driveDefaultSpeed;
                robot.rightPowerControlBack = -turnInput * robot.driveDefaultSpeed;

                robot.setDrivesByPower();
                waitMillis(22);

                robot.leftPowerControl = 0;
                robot.rightPowerControl = 0;
                robot.leftPowerControlBack = 0;
                robot.rightPowerControlBack = 0;
                robot.setDrivesByPower();
            }

            // ********************************  control:  ***********************************//
            if (gamepad1.dpad_up) {
                double xInput = 0.3;

                robot.leftDistanceControl = 0;
                robot.rightDistanceControl = 0;
                robot.leftDistanceControlBack = 0;
                robot.rightDistanceControlBack = 0;

                robot.leftPowerControl = xInput * robot.driveDefaultSpeed;
                robot.rightPowerControl = xInput * robot.driveDefaultSpeed;
                robot.leftPowerControlBack = xInput * robot.driveDefaultSpeed;
                robot.rightPowerControlBack = xInput * robot.driveDefaultSpeed;

                robot.setDrivesByPower();
                waitMillis(22);

                robot.leftPowerControl = 0;
                robot.rightPowerControl = 0;
                robot.leftPowerControlBack = 0;
                robot.rightPowerControlBack = 0;
                robot.setDrivesByPower();
            }

            // ********************************  control:  ***********************************//
            if (gamepad1.dpad_down) {
                double xInput = -0.3;

                robot.leftDistanceControl = 0;
                robot.rightDistanceControl = 0;
                robot.leftDistanceControlBack = 0;
                robot.rightDistanceControlBack = 0;

                robot.leftPowerControl = xInput * robot.driveDefaultSpeed;
                robot.rightPowerControl = xInput * robot.driveDefaultSpeed;
                robot.leftPowerControlBack = xInput * robot.driveDefaultSpeed;
                robot.rightPowerControlBack = xInput * robot.driveDefaultSpeed;

                robot.setDrivesByPower();
                waitMillis(22);

                robot.leftPowerControl = 0;
                robot.rightPowerControl = 0;
                robot.leftPowerControlBack = 0;
                robot.rightPowerControlBack = 0;
                robot.setDrivesByPower();
            }

            // ********************************  control:  ***********************************//
            if (gamepad2.dpad_right) {
                robot.moveLift(-1);
            }

            // ********************************  control:   ***********************************//
            if (gamepad2.dpad_left) {
                robot.moveLift(1);

            }

            // ********************************  control:  ***********************************//
            if (gamepad2.dpad_up) {
                robot.moveLift(0.5);

            }

            // ********************************  control:  ***********************************//
            if (gamepad2.dpad_down) {
                robot.moveLift(-0.5);

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

        String team = robot.blueTeam ? "blue" : "red";

        telemetry.addData("left drive power", "%.2f", robot.leftPowerControl);
        telemetry.addData("right drive power", "%.2f", robot.rightPowerControl);
        telemetry.addData("left drive back power", "%.2f", robot.leftPowerControlBack);
        telemetry.addData("right drive back power", "%.2f", robot.rightPowerControlBack);

        telemetry.addData("lift", "%.0f%%", robot.liftControl * 100);
        telemetry.addData("left claw", "%.0f%%", robot.leftClawControl * 100);
        telemetry.addData("right claw", "%.0f%%", robot.rightClawControl * 100);

        telemetry.addData("set heading", "%.2fdeg", (double) robot.headingControl);
        telemetry.addData("start heading", "%.2fdeg", robot.gameStartHeading);
        telemetry.addData("team", team);

        telemetry.update();
    }

    // ************************** OP END *********************************************************//
}

