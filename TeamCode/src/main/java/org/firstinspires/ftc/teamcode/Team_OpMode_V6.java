package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//********************************* MAIN OP CLASS ************************************************//

@TeleOp(name = "Team V6", group = "Team")
// @Disabled
public class Team_OpMode_V6 extends LinearOpMode {

    //********************************* HW VARIABLES *********************************************//
    private Team_Hardware_V3 robot = new Team_Hardware_V3();

    //********************************* MOVE STATES **********************************************//
    private ElapsedTime totalRuntime = null;

    //********************************* MOVE STATES **********************************************//
    private double leftDriveControl = 0;
    private double rightDriveControl = 0;
    private double headingControl = 0;
    private double liftControl = 0;
    private double leftClawControl = 0;
    private double rightClawControl = 0;
    private double baseControl = 0;
    private double elbowControl = 0;
    private double gameStartHeading = 0;

    //********************************* CONSTANTS ************************************************//
    private Boolean manualMode = true;
    private Boolean blueTeam = true;
    private Boolean rightField = true;
    private Boolean armEnabled = false;
    private Boolean loaded = false;

    //********************************* PREDEFINED POS *******************************************//
    private double clawClosed[] = {0.95, 0.10};
    private double clawZero[] = {0.22, 0.75};
    private double clawOpen[] = {0.75, 0.20};
    private double driveDefaultSpeed = 0.44;
    private double turnDefaultSpeed = 0.22;
    private double servoDefaultSpeed = 0.00033;

    // ************************** MAIN LOOP ******************************************************//

    @Override
    public void runOpMode() {

        //********************************* MAIN LOOP INIT ***************************************//
        robot.init(hardwareMap);

        ElapsedTime controlRuntime = new ElapsedTime();
        ElapsedTime loopRuntime = new ElapsedTime();
        totalRuntime = new ElapsedTime();

        controlRuntime.reset();

        robot.modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating gyro", "... do NOT move");
            telemetry.addData("calibrating gyro", "%s", Math.round(controlRuntime.seconds()));
            loaded = true;
            telemetry.update();
            sleep(66);
        }

        while (!isStopRequested() && !gamepad1.start) {

            String mode = manualMode ? "manual" : "autonomous";
            String field = rightField ? "right" : "left";
            String team = blueTeam ? "blue" : "red";

            telemetry.addData("1 ", "press START to continue");
            telemetry.addData("2 ", "select MODE");
            telemetry.addData("3 ", "(a) for manual or (b) for autonomous: " + mode);
            telemetry.addData("4 ", "(x) for blue tem or (y) for red team: " + team);
            telemetry.addData("5 ", "(r bumper) for right field or (l bumper) for left field: ", field);
            telemetry.update();

            if (gamepad1.a) manualMode = true;
            if (gamepad1.b) manualMode = false;

            if (gamepad1.x) blueTeam = true;
            if (gamepad1.y) blueTeam = false;

            if (blueTeam) robot.colorBeacon.blue();
            if (!blueTeam) robot.colorBeacon.red();

            if (gamepad1.right_bumper) rightField = true;
            if (gamepad1.left_bumper) rightField = false;
        }

        telemetry.clear();
        telemetry.update();
        telemetry.addData("driver", "CLICK  >>> to START");
        telemetry.update();

        waitForStart();

        armEnabled = false;
        leftDriveControl = 0;
        rightDriveControl = 0;
        leftClawControl = clawZero[0];
        rightClawControl = clawZero[1];
        baseControl = 6;
        elbowControl = 34;
        setDrives();
        setServos();
        robot.base.setPosition(baseControl);
        robot.elbow.setPosition(elbowControl);

        while (opModeIsActive() && loaded) {

            //********************************* CONTROL LOOP *************************************//

            double crrLoopTime = loopRuntime.nanoseconds() / 1000000; // covert to millis
            loopRuntime.reset();

            updateTelemetry();

            //********************************* MANUAL MODE **************************************//
            //              BOTH GAME PADS ARE MAPPED THE SAME AT THIS TIME                       //
            //            IF THE SAME COMMAND IS GIVEN ON BOTH GAME PAD 1 WINS                    //
            //********************************* MANUAL MODE **************************************//

            if (manualMode) {

                // ***************************** control: DRIVES  ********************************//
                {
                    double xInput = 0;
                    double yInput = 0;

                    if (Math.abs(gamepad2.left_stick_y) > 0.15)
                        yInput = -gamepad2.left_stick_y;
                    if (Math.abs(gamepad2.left_stick_x) > 0.15)
                        xInput = gamepad2.left_stick_x;

                    if (Math.abs(gamepad1.left_stick_y) > 0.15)
                        yInput = -gamepad1.left_stick_y;
                    if (Math.abs(gamepad1.left_stick_x) > 0.15)
                        xInput = gamepad1.left_stick_x;

                    leftDriveControl = yInput * driveDefaultSpeed;
                    rightDriveControl = yInput * driveDefaultSpeed;

                    leftDriveControl += xInput * turnDefaultSpeed;
                    rightDriveControl -= xInput * turnDefaultSpeed;

                    setDrives();

                    if (xInput != 0) headingControl = robot.modernRoboticsI2cGyro.getHeading();
                }

                // ********************************  control: LIFT  ******************************//
                {
                    double liftInput = 0;

                    if (Math.abs(gamepad2.right_stick_y) > 0.15)
                        liftInput = gamepad2.right_stick_y;

                    if (Math.abs(gamepad1.right_stick_y) > 0.15)
                        liftInput = gamepad1.right_stick_y;

                    double liftDefaultSpeed = 0.66;
                    liftControl = liftInput * liftDefaultSpeed;
                    setDrives();
                }

                // ********************************  control: TURNS 90  **************************//
                if (gamepad1.dpad_right ||
                        gamepad2.dpad_right) {
                    turn(-80);
                }

                // ********************************  control: TURNS -90
                if (gamepad1.dpad_left ||
                        gamepad2.dpad_left) {
                    turn(80);
                }

                // ********************************  control: TURN FACING THE CRYPTO BOX  ********//
                if (gamepad1.dpad_up ||
                        gamepad2.dpad_up) {
                    turnToHeading(gameStartHeading + 90);
                }

                // ********************************  control: TURNS 180  *************************//
                if (gamepad1.dpad_down ||
                        gamepad2.dpad_down) {
                    turn(180);
                }

                // ********************************  control: SMALL STEP FORWARD  ****************//
                if (gamepad1.y ||
                        gamepad2.y) {
                    double step = 10;

                    move(step);
                }

                // ********************************  control: SMALL STEP REVERSE  ****************//
                if (gamepad1.a ||
                        gamepad2.a) {
                    double step = 10;

                    move(-step);
                }

                // ********************************  control: SMALL STEP LEFT  *******************//
                if (gamepad1.x ||
                        gamepad2.x) {
                    double step = 10;

                    turn(-45);
                    move(-1.5 * step);
                    turn(45);
                    move(step);
                }

                // ********************************  control: SMALL STEP RIGHT  ******************//
                if (gamepad1.b ||
                        gamepad2.b) {
                    double step = 10;

                    turn(45);
                    move(-1.5 * step);
                    turn(-45);
                    move(step);
                }

                // ********************************  control: CLAW OPEN  *************************//
                if (gamepad2.left_trigger != 0) {
                    leftClawControl -= gamepad2.left_trigger * servoDefaultSpeed * crrLoopTime;
                    rightClawControl += gamepad2.left_trigger * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }
                if (gamepad1.left_trigger != 0) {
                    leftClawControl -= gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                    rightClawControl += gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }

                // ********************************  control: CLAW CLOSE  ************************//
                if (gamepad2.right_trigger != 0) {
                    leftClawControl += gamepad2.right_trigger * servoDefaultSpeed * crrLoopTime;
                    rightClawControl -= gamepad2.right_trigger * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }
                if (gamepad1.right_trigger != 0) {
                    leftClawControl += gamepad1.right_trigger * servoDefaultSpeed * crrLoopTime;
                    rightClawControl -= gamepad1.right_trigger * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }
                // ********************************  control: CLAW PREDEF OPEN  ******************//
                if (gamepad1.left_bumper ||
                        gamepad2.left_bumper) {
                    leftClawControl = clawOpen[0];
                    rightClawControl = clawOpen[1];
                    setServos();
                }
                // ********************************  control: CLAW PREDEF CLOSE  *****************//
                if (gamepad1.right_bumper ||
                        gamepad2.right_bumper) {
                    leftClawControl = clawClosed[0];
                    rightClawControl = clawClosed[1];
                    setServos();
                }
            }

            //********************************* AUTO MODE ****************************************//

            if (!manualMode) {
                runAutonomous();
            }

            //********************************* END LOOP *****************************************//
        }
    }

    //********************************* AUTO MODE HELPER FUNCTION ********************************//

    private void runAutonomous() {
        //TODO
        //deployed position
        moveArm(75, 75);

        //home position
        moveArm(0, 17);

        move(200);
        turn(90);
        waitMillis(555);

        move(200);
        turn(-90);
        waitMillis(555);

        checkAndStopAutonomous();

        move(400);
        turn(180);
        waitMillis(555);

        move(200);
        turn(-90);
        waitMillis(555);

        turnToHeading(gameStartHeading);

        stopRobot();
        stop(); //stop the opMode
    }

    // ************************** MANUAL DRIVE HELPER FUNCTIONS  *********************************//

    private void turnToHeading(double newHeading) {

        newHeading %= 360;

        double crrHeading = robot.modernRoboticsI2cGyro.getHeading();
        double turnDeg;
        double diffAbs = Math.abs(newHeading - crrHeading);
        double diff360Abs = 360 - diffAbs;
        double direction = newHeading > crrHeading ? 1.0 : -1.0;
        double inverted = diffAbs < diff360Abs ? 1.0 : -1.0;

        turnDeg = inverted > 0 ? diffAbs : diff360Abs;
        turnDeg *= direction * inverted;

        turn(turnDeg);
    }

    private void turn(double turnDeg) {

        double breakDistance = 15; // ramp down the last 15 deg
        double maxPower = 0.44;
        double minPower = 0.10;

        double direction = turnDeg > 0 ? 1.0 : -1.0;
        double crrError = turnDeg * direction;
        double startHeading = robot.modernRoboticsI2cGyro.getHeading();
        double endHeading = startHeading + turnDeg;

        while (crrError > 3) {

            double crrPower = maxPower;

            if (crrError < breakDistance) {
                crrPower = minPower + crrError / (maxPower - minPower);
            }

            leftDriveControl = -crrPower * direction;
            rightDriveControl = crrPower * direction;

            setDrives();
            waitMillis(1);

            double crrHeading = robot.modernRoboticsI2cGyro.getHeading();

            if (startHeading >= 180 && direction > 0 && crrHeading < 180) {
                crrHeading += 360;
            }
            if (startHeading <= 180 && direction < 0 && crrHeading > 180) {
                crrHeading -= 360;
            }
            crrError = (endHeading - crrHeading) * direction;
        }
        stopRobot();
        headingControl = robot.modernRoboticsI2cGyro.getHeading();
    }

    private void move(double distance) {

        double movePerMs = 0.01; // how much it moves in 1ms at max power
        double breakDistance = 22; //ramp down the last 22 mm

        double maxPower = 0.44;
        double minPower = 0.10;

        double direction = distance > 0 ? 1.0 : -1.0;
        double crrError = distance * direction;

        while (crrError > 3) {

            double crrPower = maxPower;

            if (crrError < breakDistance) {
                crrPower = minPower + crrError / (maxPower - minPower);
            }

            leftDriveControl = crrPower * direction; //power to motors is proportional with the speed
            rightDriveControl = crrPower * direction;

            setDrives();
            waitMillis(1);

            crrError -= crrPower * movePerMs;
        }
        stopRobot();
    }

    // ************************** ARM  DRIVE SERVOS HELPER FUNCTIONS  ****************************//

    private void moveArm(double newBase, double newElbow) {

        if (!armEnabled) {
            return;
        }

        double stepSize = 0.01;

        double baseStart = baseControl;
        double elbowStart = elbowControl;

        double stepCount = Math.abs(baseStart - newBase) / stepSize;
        stepCount = Math.max(Math.abs(elbowStart - newElbow) / stepSize, stepCount);
        stepCount = Range.clip(stepCount, 0, 333); //should not be more than 1000 steps = 62secs

        double elbowStepSize = (newElbow - baseStart) / stepCount;
        double baseStepSize = (newBase - elbowStart) / stepCount;

        for (int i = 0; i < (int) stepCount; i++) {

            double baseCrr = baseStart + i * baseStepSize;
            double elbowCrr = elbowStart + i * elbowStepSize;

            baseControl = baseCrr;
            elbowControl = elbowCrr;
            setServos();

            waitMillis(11); //TODO
        }

        baseControl = newBase;
        elbowControl = newElbow;
        setServos();
    }

    // ************************** HARDWARE SET FUNCTIONS *****************************************//

    private void setDrives() {

        leftDriveControl = Range.clip(leftDriveControl, -0.88, 0.88);
        rightDriveControl = Range.clip(rightDriveControl, -0.88, 0.88);

        liftControl = Range.clip(liftControl, -0.66, 0.66);

//       if (liftControl >= 0 && !robot.topSwitch.getState()) { // false means switch is pressed
//           liftControl = 0;
//        }
//        if (liftControl < 0 && !robot.bottomSwitch.getState()) { // false means switch is pressed
//            liftControl = 0;
//        }
        if (liftControl < 0) {
            robot.liftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.liftDrive.setPower(-liftControl);
        }
        if (liftControl >= 0) {
            robot.liftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.liftDrive.setPower(liftControl);
        }

        // if gyro indicates drifting add same power to correct
        // do not add if already doing a turn
        double headingCorrection = 0;

        if (leftDriveControl == rightDriveControl) {
            double headingCrr = robot.modernRoboticsI2cGyro.getHeading();
            double error;
            double driveDirection = leftDriveControl > 0 ? 1.0 : -1.0;

            if (headingControl > 180 && headingCrr <= 180 && headingControl - headingCrr > 180) {
                headingCrr += 360;
            }
            if (headingControl <= 180 && headingCrr > 180 && headingCrr - headingControl > 180) {
                headingCrr -= 360;
            }
            error = headingCrr - headingControl;
            error = Range.clip(error, -45, 45); // if completely off trim down

            headingCorrection = error / 45 * 0.05; //TODO tune up the amount of correction
            headingCorrection *= driveDirection; // apply correction the other way when running in reverse

            if (leftDriveControl == 0) headingCorrection = 0;
        }

        if (leftDriveControl >= 0) {
            robot.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftDrive.setPower(Math.abs(leftDriveControl) - headingCorrection);
        }
        if (leftDriveControl < 0) {
            robot.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.leftDrive.setPower(Math.abs(leftDriveControl) - headingCorrection);
        }
        if (rightDriveControl >= 0) {
            robot.rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.rightDrive.setPower(Math.abs(rightDriveControl) + headingCorrection);
        }
        if (rightDriveControl < 0) {
            robot.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive.setPower(Math.abs(rightDriveControl) + headingCorrection);
        }
    }

    private void setServos() {

        double minLeftClaw = Math.min(Math.min(clawZero[0], clawClosed[0]), clawOpen[0]);
        double maxLeftClaw = Math.max(Math.max(clawZero[0], clawClosed[0]), clawOpen[0]);

        double minRightClaw = Math.min(Math.min(clawZero[1], clawClosed[1]), clawOpen[1]);
        double maxRightClaw = Math.max(Math.max(clawZero[1], clawClosed[1]), clawOpen[1]);

        leftClawControl = Range.clip(leftClawControl, minLeftClaw, maxLeftClaw);
        rightClawControl = Range.clip(rightClawControl, minRightClaw, maxRightClaw);

        robot.leftClaw.setPosition(leftClawControl);
        robot.rightClaw.setPosition(rightClawControl);

        if (armEnabled) {
            robot.base.setPosition(baseControl);
            robot.elbow.setPosition(elbowControl);
        }
    }

    // ************************** GENERAL MOVE HELPER FUNCTIONS  *********************************//

    private void checkAndStopAutonomous() {

        if (manualMode) {
            return;
        }

        if (stopTime(30)) {
            stopRobot();
            stop(); //stop the opMode
        }
    }

    private void stopRobot() {
        leftDriveControl = 0;
        rightDriveControl = 0;
        setDrives();
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

        String mode = manualMode ? "manual" : "autonomous";
        String field = rightField ? "right" : "left";
        String team = blueTeam ? "blue" : "red";

        telemetry.addData("left drive", "%.0f%%", leftDriveControl * 100);
        telemetry.addData("right drive", "%.0f%%", rightDriveControl * 100);
        telemetry.addData("lift", "%.0f%%", liftControl * 100);
        telemetry.addData("left claw", "%.0f%%", leftClawControl * 100);
        telemetry.addData("right claw", "%.0f%%", rightClawControl * 100);

        telemetry.addData("crr heading", "%.2fdeg", (double) robot.modernRoboticsI2cGyro.getHeading());
        telemetry.addData("set heading", "%.2fdeg", (double) headingControl);
        telemetry.addData("start heading", "%.2fdeg", gameStartHeading);
        telemetry.addData("z angle", "%.2fdeg",
                (double) robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("mode", mode);
        telemetry.addData("team", team);
        telemetry.addData("field", field);
        //telemetry.addData("total runtime", "%.0fs", totalRuntime.seconds());

        telemetry.update();
    }

    // ************************** OP END *********************************************************//
}

