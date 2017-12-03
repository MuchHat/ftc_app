package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//********************************* MAIN OP CLASS ************************************************//

@TeleOp(name = "Manual Blue V9", group = "Manual")
// @Disabled
public class Manual_Blue_V9 extends LinearOpMode {

    //********************************* HW VARIABLES *********************************************//
    private Team_Hardware_V9 robot = new Team_Hardware_V9();

    //********************************* MOVE STATES **********************************************//
    private ElapsedTime totalRuntime = null;

    double liftMoving = 0;
    double timeLeftLiftMoving = 0;

    // ************************** MAIN LOOP ******************************************************//

    @Override
    public void runOpMode() {

        //********************************* MAIN LOOP INIT ***************************************//

        telemetry.addData("DRIVER", ">>> WAIT WAIT WAIT >>>");
        telemetry.update();

        robot.init(hardwareMap);

        ElapsedTime controlRuntime = new ElapsedTime();
        ElapsedTime loopRuntime = new ElapsedTime();
        totalRuntime = new ElapsedTime();

        controlRuntime.reset();
        robot.blueTeam = true;
        robot.showTeamColor();

        telemetry.addData("DRIVER", ">>>  PRESS START >>>");
        telemetry.update();

        waitForStart();
        robot.imuGyro.start();

        liftMoving = 0;
        timeLeftLiftMoving = 0;
        robot.openClawZero();

        while (opModeIsActive()) {

            //********************************* CONTROL LOOP *************************************//

            double crrLoopTime = loopRuntime.nanoseconds() / 1000000; // covert to millis
            loopRuntime.reset();

            updateTelemetry();

            // ***************************** control: DRIVES AND SIDE  ***************************//
            {
                double xInput = 0;
                double yInput = 0;
                double sideInput = 0;

                robot.leftDistanceControl = 0;
                robot.rightDistanceControl = 0;
                robot.leftDistanceControlBack = 0;
                robot.rightDistanceControlBack = 0;

                if (Math.abs(gamepad1.right_stick_x) > 0.15) {
                    sideInput = gamepad1.right_stick_x;
                }
                if (Math.abs(gamepad1.left_stick_y) > 0.03) {
                    yInput = -gamepad1.left_stick_y;
                }
                if (Math.abs(gamepad1.left_stick_x) > 0.03) {
                    xInput = gamepad1.left_stick_x;
                }

                double leftPower = 0;
                double rightPower = 0;
                double leftPowerBack = 0;
                double rightPowerBack = 0;

                leftPower += -sideInput * robot.driveDefaultSpeed;
                rightPower += sideInput * robot.driveDefaultSpeed;
                leftPowerBack += sideInput * robot.driveDefaultSpeed;
                rightPowerBack += -sideInput * robot.driveDefaultSpeed;

                leftPower += yInput * robot.driveDefaultSpeed;
                rightPower += yInput * robot.driveDefaultSpeed;
                leftPower += xInput * robot.turnDefaultSpeed;
                rightPower -= xInput * robot.turnDefaultSpeed;

                leftPowerBack += yInput * robot.driveDefaultSpeed;
                rightPowerBack += yInput * robot.driveDefaultSpeed;
                leftPowerBack += xInput * robot.turnDefaultSpeed;
                rightPowerBack -= xInput * robot.turnDefaultSpeed;

                robot.leftPowerControl = leftPower;
                robot.rightPowerControl = rightPower;
                robot.leftPowerControlBack = leftPowerBack;
                robot.rightPowerControlBack = rightPowerBack;

                robot.setDrivesByPower();
            }

            // ********************************  control: LIFT  ******************************//

            {
                if (gamepad1.a) {
                    timeLeftLiftMoving = 111;
                    liftMoving = -1;
                }

                if (gamepad1.y) {
                    timeLeftLiftMoving = 111;
                    liftMoving = 1;
                }

                if (timeLeftLiftMoving > 0) {

                    timeLeftLiftMoving -= crrLoopTime;
                    timeLeftLiftMoving = Range.clip(timeLeftLiftMoving, 0, 9999);

                    robot.liftControl = liftMoving;
                    robot.setDrivesByPower();
                } else {
                    liftMoving = 0;
                }
            }

            // ********************************  control: LIFT  ******************************//

            {
                double liftInput = liftMoving;

                if (Math.abs(gamepad2.right_stick_y) > 0.06) {
                    liftInput = -gamepad2.right_stick_y;
                }

                if (Math.abs(gamepad1.right_stick_y) > 0.06) {
                    liftInput = -gamepad1.right_stick_y;
                }

                robot.liftControl = liftInput;
                robot.setDrivesByPower();
            }

            // ********************************  control: STEPS *********************************//

            if (gamepad1.dpad_right) {

                double frontInput = 0.66;
                double backInput = 0.22;

                robot.leftDistanceControl = 0;
                robot.rightDistanceControl = 0;
                robot.leftDistanceControlBack = 0;
                robot.rightDistanceControlBack = 0;

                robot.leftPowerControl = -frontInput;
                robot.rightPowerControl = frontInput;
                robot.leftPowerControlBack = backInput;
                robot.rightPowerControlBack = -backInput;

                robot.setDrivesByPower();
                waitMillis(666);

                robot.leftPowerControl = 0;
                robot.rightPowerControl = 0;
                robot.leftPowerControlBack = 0;
                robot.rightPowerControlBack = 0;
                robot.setDrivesByPower();
            }

            // ********************************   control: STEPS *********************************//
            if (gamepad1.dpad_left) {

                double frontInput = -0.66;
                double backInput = -0.22;

                robot.leftDistanceControl = 0;
                robot.rightDistanceControl = 0;
                robot.leftDistanceControlBack = 0;
                robot.rightDistanceControlBack = 0;

                robot.leftPowerControl = -frontInput;
                robot.rightPowerControl = frontInput;
                robot.leftPowerControlBack = backInput;
                robot.rightPowerControlBack = -backInput;

                robot.setDrivesByPower();
                waitMillis(666);

                robot.leftPowerControl = 0;
                robot.rightPowerControl = 0;
                robot.leftPowerControlBack = 0;
                robot.rightPowerControlBack = 0;
                robot.setDrivesByPower();
            }

            // ********************************  control: STEPS *********************************//
            if (gamepad1.dpad_up) {

                double xInput = 0.33;

                robot.leftDistanceControl = 0;
                robot.rightDistanceControl = 0;
                robot.leftDistanceControlBack = 0;
                robot.rightDistanceControlBack = 0;

                robot.leftPowerControl = xInput * robot.driveDefaultSpeed;
                robot.rightPowerControl = xInput * robot.driveDefaultSpeed;
                robot.leftPowerControlBack = xInput * robot.driveDefaultSpeed;
                robot.rightPowerControlBack = xInput * robot.driveDefaultSpeed;

                robot.setDrivesByPower();
                waitMillis(222);

                robot.leftPowerControl = 0;
                robot.rightPowerControl = 0;
                robot.leftPowerControlBack = 0;
                robot.rightPowerControlBack = 0;
                robot.setDrivesByPower();
            }

            // ********************************  control: STEPS *********************************//
            if (gamepad1.dpad_down) {

                double xInput = -0.33;

                robot.leftDistanceControl = 0;
                robot.rightDistanceControl = 0;
                robot.leftDistanceControlBack = 0;
                robot.rightDistanceControlBack = 0;

                robot.leftPowerControl = xInput * robot.driveDefaultSpeed;
                robot.rightPowerControl = xInput * robot.driveDefaultSpeed;
                robot.leftPowerControlBack = xInput * robot.driveDefaultSpeed;
                robot.rightPowerControlBack = xInput * robot.driveDefaultSpeed;

                robot.setDrivesByPower();
                waitMillis(222);

                robot.leftPowerControl = 0;
                robot.rightPowerControl = 0;
                robot.leftPowerControlBack = 0;
                robot.rightPowerControlBack = 0;
                robot.setDrivesByPower();
            }

            // ********************************  control: STEPS *********************************//
            if (gamepad1.b) {

                double sideInput = 0.66;

                robot.leftDistanceControl = 0;
                robot.rightDistanceControl = 0;
                robot.leftDistanceControlBack = 0;
                robot.rightDistanceControlBack = 0;

                robot.leftPowerControl = -sideInput;
                robot.rightPowerControl = sideInput;
                robot.leftPowerControlBack = sideInput;
                robot.rightPowerControlBack = -sideInput;

                robot.setDrivesByPower();
                waitMillis(222);

                robot.leftPowerControl = 0;
                robot.rightPowerControl = 0;
                robot.leftPowerControlBack = 0;
                robot.rightPowerControlBack = 0;

                robot.setDrivesByPower();
            }

            // ********************************  control: STEPS *********************************//
            if (gamepad1.x) {

                double sideInput = -0.66;

                robot.leftDistanceControl = 0;
                robot.rightDistanceControl = 0;
                robot.leftDistanceControlBack = 0;
                robot.rightDistanceControlBack = 0;

                robot.leftPowerControl = -sideInput;
                robot.rightPowerControl = sideInput;
                robot.leftPowerControlBack = sideInput;
                robot.rightPowerControlBack = -sideInput;

                robot.setDrivesByPower();
                waitMillis(222);

                robot.leftPowerControl = 0;
                robot.rightPowerControl = 0;
                robot.leftPowerControlBack = 0;
                robot.rightPowerControlBack = 0;

                robot.setDrivesByPower();
            }

            // ********************************  control: CLAW OPEN  ****************************//
            if (Math.abs(gamepad1.right_trigger) > 0.03) {

                robot.leftClawControl -= gamepad1.right_trigger * robot.servoDefaultSpeed * crrLoopTime;
                robot.rightClawControl += gamepad1.right_trigger * robot.servoDefaultSpeed * crrLoopTime;
                robot.setServos();
            }

            // ********************************  control: CLAW CLOSE  ***************************//
            if (Math.abs(gamepad1.left_trigger) > 0.03) {

                robot.leftClawControl += gamepad1.left_trigger * robot.servoDefaultSpeed * crrLoopTime;
                robot.rightClawControl -= gamepad1.left_trigger * robot.servoDefaultSpeed * crrLoopTime;
                robot.setServos();
            }

            // ********************************  control: CLAW PREDEF OPEN  **********************//
            if (gamepad1.left_bumper) {
                if (robot.isClawOpened()) {
                    robot.openClawWide();
                    waitMillis(111);
                } else {
                    robot.openClaw();
                    waitMillis(111);
                }
            }

            // ********************************  control: CLAW PREDEF CLOSE  ********************//
            if (gamepad1.right_bumper) {
                robot.closeClaw();
            }

            //********************************* END LOOP *****************************************//
        }

        robot.stopRobot();
        robot.colorBeacon.off();
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

        telemetry.addData("team", team);

        telemetry.update();
    }

    // ************************** OP END *********************************************************//
}

