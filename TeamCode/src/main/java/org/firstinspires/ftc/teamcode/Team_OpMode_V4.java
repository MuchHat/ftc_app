/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//********************************* OP CLASS*** **************************************************//

@TeleOp(name = "Team V4", group = "Team")
// @Disabled
public class Team_OpMode_V4 extends LinearOpMode {

    //********************************* HW VARIABLES *********************************************//

    public Team_Hardware_V2 robot = new Team_Hardware_V2();

    private ElapsedTime loopRuntime = new ElapsedTime();
    private ElapsedTime controlRuntime = new ElapsedTime();

    //********************************* MOVE STATE ***********************************************//

    private ElapsedTime totalRuntime = new ElapsedTime();
    private double leftDriveControl = 0;
    private double rightDriveControl = 0;
    private double headingControl = 0;
    private double liftControl = 0;
    private double leftClawControl = 0;
    private double rightClawControl = 0;
    private double baseControl = 0;
    private double elbowControl = 0;
    private double gameStartHeading = 0;
    private Boolean armEnabled = false;

    //********************************* CONSTANTS ************************************************//

    private Boolean manualMode = true;
    private Boolean blueTeam = true;
    private Boolean rightField = true;

    //********************************* CONSTANTS ************************************************//

    private double driveDefaultSpeed = 0.44; // TODO
    private double turnDefaultSpeed = 0.22; // TODO
    private double liftDefaultSpeed = 0.22; // TODO
    private double servoDefaultSpeed = 0.00033; // TODO
    private double robotLinearMoveMillis = 1.0; // moves 1mm in 1ms at 1.0 power
    private double robotAngularMoveMillis = 1.0; // moves 1deg in 1ms at 1.0 power

    //********************************* PREDEF POS ***********************************************//

    private double clawOpen[] = {0.76, 0.44};
    private double clawClosed[] = {0.85, 0.34};

    // ************************** AUTO MODE ******************************************************//

    @Override
    public void runOpMode() {

        //********************************* INIT LOOP ********************************************//
        robot.init(hardwareMap);

        telemetry.log().add("calibrating gyro ... do not move");
        telemetry.update();
        sleep(444);
        robot.modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        controlRuntime.reset();
        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating gyro", "%s", Math.round(controlRuntime.seconds()));
            telemetry.update();
            sleep(66);
        }
        controlRuntime.reset();

        while (!isStopRequested() && !gamepad1.start) {

            String mode = manualMode ? "manual" : "autonomous";
            String field = rightField ? "right" : "left";
            String team = blueTeam ? "blue" : "red";

            telemetry.log().add("select mode");
            telemetry.log().add("(a) for manual or (b) for autonomous: " + mode);
            telemetry.log().add("(x) for blue tem or (y) for red team: " + team);
            telemetry.log().add("(r bumper) for right field or (l bumper) for left field: ", field);
            telemetry.log().add("press start to continue");
            telemetry.update();

            if (gamepad1.a) manualMode = true;
            if (gamepad1.b) manualMode = false;

            if (gamepad1.x) blueTeam = true;
            if (gamepad1.y) blueTeam = false;

            if (gamepad1.right_bumper) rightField = true;
            if (gamepad1.left_bumper) rightField = false;
        }

        setDrives();
        setServos();

        updateTelemetry();

        waitForStart();
        loopRuntime.reset();
        totalRuntime.reset();
        controlRuntime.reset();

        while (opModeIsActive()) {

            //********************************* START LOOP *****************************************

            double crrLoopTime = loopRuntime.nanoseconds() / 1000000; // covert to millis
            loopRuntime.reset();

            updateTelemetry();

            //********************************* MANUAL MODE *****************************************

            if (manualMode) {

                // ***************************** control: DRIVES
                {
                    double xInput = 0;
                    double yInput = 0;

                    if (Math.abs(gamepad1.left_stick_x) > 0.15)
                        xInput = gamepad1.left_stick_x; //TODO
                    if (Math.abs(gamepad1.left_stick_y) > 0.15)
                        yInput = -gamepad1.left_stick_y; //TODO

                    leftDriveControl = yInput * driveDefaultSpeed;
                    rightDriveControl = yInput * driveDefaultSpeed;

                    leftDriveControl += xInput * turnDefaultSpeed;
                    rightDriveControl -= xInput * turnDefaultSpeed;

                    setDrives();

                    if (xInput != 0) headingControl = robot.modernRoboticsI2cGyro.getHeading();
                }

                // ********************************  control: LIFT
                {
                    double liftInput = 0;

                    if (Math.abs(gamepad1.right_stick_y) > 0.15)
                        liftInput = gamepad1.right_stick_y; //TODO

                    liftControl = liftInput * liftDefaultSpeed;
                    setDrives();
                }

                // ********************************  control: TURNS 90
                if (gamepad1.dpad_right) {
                    turn(90);
                }

                // ********************************  control: TURNS -90
                if (gamepad1.dpad_left) {
                    turn(-90);
                }

                // ********************************  control: TURN FACING THE CRYPTO BOX
                if (gamepad1.dpad_up) {
                    turnToHeading(gameStartHeading + 90);
                }

                // ********************************  control: TURNS 180
                if (gamepad1.dpad_down) {
                    turn(180);
                }

                // ********************************  control: SMALL STEP FORWARD
                if (gamepad1.y) {
                    double step = 10;

                    move(step);
                }

                // ********************************  control: SMALL STEP REVERSE
                if (gamepad1.a) {
                    double step = 10;

                    move(-step);
                }

                // ********************************  control: SMALL STEP LEFT
                if (gamepad1.x) {
                    double step = 10;

                    turn(-45);
                    move(-1.5 * step);
                    turn(45);
                    move(step);
                }

                // ********************************  control: SMALL STEP RIGHT
                if (gamepad1.b) {
                    double step = 10;

                    turn(45);
                    move(-1.5 * step);
                    turn(-45);
                    move(step);
                }

                // ********************************  control: CLAW OPEN
                if (gamepad1.left_trigger != 0) {
                    leftClawControl -= gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                    rightClawControl += gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }

                // ********************************  control: CLAW CLOSE
                if (gamepad1.right_trigger != 0) {
                    leftClawControl += gamepad1.right_trigger * servoDefaultSpeed * crrLoopTime;
                    rightClawControl -= gamepad1.right_trigger * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }
                // ********************************  control: CLAW PREDEF OPEN
                if (gamepad1.left_bumper) {
                    leftClawControl = clawOpen[0];
                    rightClawControl = clawOpen[1];
                    setServos();
                }
                // ********************************  control: CLAW PREDEF CLOSE
                if (gamepad1.right_bumper) {
                    leftClawControl = clawClosed[0];
                    rightClawControl = clawClosed[1];
                    setServos();
                }
                // ********************************  control: LOAD CUBE SEQUENCE
                if (gamepad1.left_stick_x > 0.15) {
                    // TODO

                }
                // ********************************  control: UNLOAD CUBE SEQUENCE
                if (gamepad1.left_stick_x < -0.15) {
                    //TODO

                }
            }

            //********************************* AUTO MODE *****************************************

            if (!manualMode) {
                runAutonomous();
            }

            //********************************* END LOOP *****************************************//
        }
    }

    private void runAutonomous() {

        //example
        //TODO

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

    // ************************** DRIVES HELPER FUNCTIONS  ***************************************//

    private void turnToHeading(double newHeading) {

        newHeading %= 360;

        double crrHeading = robot.modernRoboticsI2cGyro.getHeading();
        double turnDeg = newHeading - crrHeading;

        if (Math.abs(newHeading - crrHeading) > 360 - Math.abs(newHeading - crrHeading)) {
            double direction = newHeading > crrHeading ? 1.0 : -1.0;

            turnDeg = 360 - Math.abs(newHeading - crrHeading);
            turnDeg *= direction * -1.0;
        }

        turn(turnDeg);
    }

    private void turn(double turnDeg) {

        double startHeading = robot.modernRoboticsI2cGyro.getHeading();
        double endHeading = (startHeading + turnDeg + 360) % 360;
        endHeading = Range.clip(endHeading, 0, 360);

        double direction = 1.0;
        double inverted = 1.0;
        double diffAbs = Math.abs(endHeading - startHeading);
        double diff360Abs = 360 - diffAbs;
        direction = endHeading > startHeading ? 1.0 : -1.0;
        if (diff360Abs < diffAbs) {
            direction *= -1;
            inverted = -1.0;
        }

        double distanceAbs = Math.min(diffAbs, diff360Abs);

        Animator turnAnimator = new Animator();
        turnAnimator.configRamp(333, 444);
        turnAnimator.configSpeed(0.04, 0.88, 0.1, 1);
        turnAnimator.start(0, distanceAbs * inverted * direction);

        turnAnimator.advanceStep(0);
        double nextSpeedAbs = turnAnimator.getSpeed();

        while (nextSpeedAbs > 0) {

            leftDriveControl = nextSpeedAbs * direction; //power to motors is proportional with the speed
            rightDriveControl = -nextSpeedAbs * direction;

            setDrives();
            waitMillis(1);

            double crrHeading = robot.modernRoboticsI2cGyro.getHeading();
            double crrDiffAbs = Math.abs(endHeading - crrHeading);
            double crrDiff360Abs = 360 - diffAbs;
            double crrDistance = Math.min(crrDiffAbs, crrDiff360Abs) * inverted * direction;

            turnAnimator.advanceStep(crrDistance);
            nextSpeedAbs = turnAnimator.getSpeed();
        }
        stopRobot();
        headingControl = robot.modernRoboticsI2cGyro.getHeading();
    }

    private void move(double distance) {

        double direction = distance > 0 ? 1.0 : -1.0;

        Animator moveAnimator = new Animator();
        moveAnimator.configRamp(333, 444);
        moveAnimator.configSpeed(0.04, 0.88, 1, 1);
        moveAnimator.start(0, Math.abs(distance));

        moveAnimator.advanceStepNoPos();
        double nextSpeed = moveAnimator.getSpeed();

        while (nextSpeed > 0) {

            leftDriveControl = nextSpeed * direction; //power to motors is proportional with the speed
            rightDriveControl = nextSpeed * direction;

            setDrives();
            waitMillis(1);

            moveAnimator.advanceStepNoPos();
            nextSpeed = moveAnimator.getSpeed();
        }
        stopRobot();
    }

    // ************************** ARM SERVOS HELPER FUNCTIONS  ************************************//

    void moveArm(double newBase, double newElbow) {

        if (!armEnabled) {
            return;
        }

        Animator baseAnimator = new Animator();
        baseAnimator.configRamp(333, 444);
        baseAnimator.configSpeed(0.04, 0.88, 0.01, 1);
        baseAnimator.start(baseControl, newBase);

        Animator elbowAnimator = new Animator();
        elbowAnimator.configRamp(333, 444);
        elbowAnimator.configSpeed(0.04, 0.88, 0.01, 1);
        elbowAnimator.start(elbowControl, newElbow);

        baseAnimator.advanceStepNoPos();
        elbowAnimator.advanceStepNoPos();
        double nextSpeedBase = baseAnimator.getSpeed();
        double nextSpeedElbow = elbowAnimator.getSpeed();

        while (nextSpeedBase > 0 || nextSpeedElbow > 0) {

            baseControl = baseAnimator.getPos();
            elbowControl = elbowAnimator.getPos();
            setServos();
            waitMillis(1);

            baseAnimator.advanceStepNoPos();
            elbowAnimator.advanceStepNoPos();
            nextSpeedBase = baseAnimator.getSpeed();
            nextSpeedElbow = elbowAnimator.getSpeed();
        }

        baseControl = newBase;
        elbowControl = newElbow;
        setServos();
    }

    // ************************** HARDWARE SET FUNCTIONS *****************************************//

    void setDrives() {

        leftDriveControl = Range.clip(leftDriveControl, -0.88, 0.88); //TODO max max power
        rightDriveControl = Range.clip(rightDriveControl, -0.88, 0.88); //TODO max max power

        liftControl = Range.clip(liftControl, -0.66, 0.66); //TODO max max power

        if (liftControl >= 0 && !robot.topSwitch.getState()) { // false means switch is pressed
            liftControl = 0;
        }
        if (liftControl < 0 && !robot.bottomSwitch.getState()) { // false means switch is pressed
            liftControl = 0;
        }
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
            double error = headingControl - robot.modernRoboticsI2cGyro.getHeading();
            double driveDirection = leftDriveControl > 0 ? 1.0 : -1.0;

            if (error > 180) error = -360 + error; // convert to +/- 180
            if (error < -180) error = -360 + error; // convert to +/- 180
            error %= 180;
            error = Range.clip(error, -45, 45); // if completely off trim down

            headingCorrection = error / 45 * 0.33; //TODO tune up the amount of correction
            headingCorrection *= driveDirection; // apply correction the other way when running in reverse
        }

        if (leftDriveControl >= 0) {
            robot.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftDrive.setPower(leftDriveControl - headingCorrection);
        }
        if (leftDriveControl < 0) {
            robot.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.leftDrive.setPower(leftDriveControl - headingCorrection);
        }
        if (rightDriveControl >= 0) {
            robot.rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.rightDrive.setPower(rightDriveControl + headingCorrection);
        }
        if (rightDriveControl < 0) {
            robot.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive.setPower(rightDriveControl + headingCorrection);
        }
    }

    void setServos() {

        leftClawControl = Range.clip(leftClawControl, 0.05, 0.95); //TODO
        rightClawControl = Range.clip(rightClawControl, 0.05, 0.95); //TODO

        robot.leftClaw.setPosition(leftClawControl);
        robot.rightClaw.setPosition(rightClawControl);
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

        millis = Range.clip(millis, 0.01, millis);
        ElapsedTime runtimeWait = new ElapsedTime();

        runtimeWait.reset();

        while (runtimeWait.nanoseconds() < millis * 1000 * 1000) {
            idle();
        }
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
        telemetry.addData("total runtime", "%.0fs", totalRuntime.seconds());

        telemetry.update();
    }

    // ************************** OP END *********************************************************//
}

