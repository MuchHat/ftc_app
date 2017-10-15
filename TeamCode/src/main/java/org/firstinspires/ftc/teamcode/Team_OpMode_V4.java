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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

//********************************* MAIN OP CLASS ************************************************//

@TeleOp(name = "Team V4", group = "Team")
@Disabled
public class Team_OpMode_V4 extends LinearOpMode {

    //********************************* HW VARIABLES *********************************************//

    Team_Hardware_V2 robot = new Team_Hardware_V2();

    ElapsedTime loopRuntime = null;
    ElapsedTime controlRuntime = null;
    ElapsedTime totalRuntime = null;
    ElapsedTime runtimeWait = null;

    //********************************* MOVE STATES **********************************************//


    double leftDriveControl = 0;
    double rightDriveControl = 0;
    double headingControl = 0;
    double liftControl = 0;
    double leftClawControl = 0;
    double rightClawControl = 0;
    double baseControl = 0;
    double elbowControl = 0;
    double gameStartHeading = 0;
    Boolean armEnabled = false;

    //********************************* CONSTANTS ************************************************//

    Boolean manualMode = true;
    Boolean blueTeam = true;
    Boolean rightField = true;

    //********************************* CONSTANTS ************************************************//

    double driveDefaultSpeed = 0.44; // TODO
    double turnDefaultSpeed = 0.22; // TODO
    double liftDefaultSpeed = 0.66; // TODO
    double servoDefaultSpeed = 0.00033; // TODO

    //********************************* PREDEFINED POS *******************************************//

    double clawOpen[] = {0.82, 0.18};
    double clawClosed[] = {0.93, 0.12};
    double clasZero[] = {0.22, 0.75};

    // ************************** MAIN LOOP ******************************************************//

    @Override
    public void runOpMode() {

        //********************************* MAIN LOOP INIT ***************************************//
        robot.init(hardwareMap);

        controlRuntime = new ElapsedTime();
        loopRuntime = new ElapsedTime();
        totalRuntime = new ElapsedTime();
        runtimeWait = new ElapsedTime();

        controlRuntime.reset();

        robot.modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating gyro", "... do NOT move");
            telemetry.addData("calibrating gyro", "%s", Math.round(controlRuntime.seconds()));
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
        leftClawControl = clasZero[0];
        rightClawControl = clasZero[1];
        baseControl = 6;
        elbowControl = 34;
        setDrives();
        setServos();
        robot.base.setPosition(baseControl);
        robot.elbow.setPosition(elbowControl);

        while (opModeIsActive()) {

            //********************************* CONTROL LOOP *************************************//

            double crrLoopTime = loopRuntime.nanoseconds() / 1000000; // covert to millis
            loopRuntime.reset();

            updateTelemetry();

            //********************************* MANUAL MODE **************************************//

            if (manualMode) {

                // ***************************** control: DRIVES  ********************************//
                {
                    double xInput = 0;
                    double yInput = 0;

                    if (Math.abs(gamepad1.right_stick_x) > 0.15)
                        xInput = gamepad1.right_stick_x; //TODO
                    if (Math.abs(gamepad1.right_stick_y) > 0.15)
                        yInput = -gamepad1.right_stick_y; //TODO

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

                    if (Math.abs(gamepad1.left_stick_y) > 0.15)
                        liftInput = gamepad1.left_stick_y; //TODO

                    liftControl = liftInput * liftDefaultSpeed;
                    setDrives();
                }

                // ********************************  control: TURNS 90  **************************//
                if (gamepad1.dpad_right) {
                    turn(90);
                }

                // ********************************  control: TURNS -90
                if (gamepad1.dpad_left) {
                    turn(-90);
                }

                // ********************************  control: TURN FACING THE CRYPTO BOX  ********//
                if (gamepad1.dpad_up) {
                    turnToHeading(gameStartHeading + 90);
                }

                // ********************************  control: TURNS 180  *************************//
                if (gamepad1.dpad_down) {
                    turn(180);
                }

                // ********************************  control: SMALL STEP FORWARD  ****************//
                if (gamepad1.y) {
                    double step = 10;

                    move(step);
                }

                // ********************************  control: SMALL STEP REVERSE  ****************//
                if (gamepad1.a) {
                    double step = 10;

                    move(-step);
                }

                // ********************************  control: SMALL STEP LEFT  *******************//
                if (gamepad1.x) {
                    double step = 10;

                    turn(-45);
                    move(-1.5 * step);
                    turn(45);
                    move(step);
                }

                // ********************************  control: SMALL STEP RIGHT  ******************//
                if (gamepad1.b) {
                    double step = 10;

                    turn(45);
                    move(-1.5 * step);
                    turn(-45);
                    move(step);
                }

                // ********************************  control: CLAW OPEN  *************************//
                if (gamepad1.left_trigger != 0) {
                    leftClawControl -= gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                    rightClawControl += gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }

                // ********************************  control: CLAW CLOSE  ************************//
                if (gamepad1.right_trigger != 0) {
                    leftClawControl += gamepad1.right_trigger * servoDefaultSpeed * crrLoopTime;
                    rightClawControl -= gamepad1.right_trigger * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }
                // ********************************  control: CLAW PREDEF OPEN  ******************//
                if (gamepad1.left_bumper) {
                    leftClawControl = clawOpen[0];
                    rightClawControl = clawOpen[1];
                    setServos();
                }
                // ********************************  control: CLAW PREDEF CLOSE  *****************//
                if (gamepad1.right_bumper) {
                    leftClawControl = clawClosed[0];
                    rightClawControl = clawClosed[1];
                    setServos();
                }
                // ********************************  control: LOAD CUBE SEQUENCE  ****************//
                if (gamepad1.left_stick_x > 0.15) {
                    // TODO

                }
                // ********************************  control: UNLOAD CUBE SEQUENCE  **************//
                if (gamepad1.left_stick_x < -0.15) {
                    //TODO

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

    void runAutonomous() {

        //example
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

    void turnToHeading(double newHeading) {

        newHeading %= 360;

        double crrHeading = robot.modernRoboticsI2cGyro.getHeading();
        double turnDeg = newHeading - crrHeading;
        double diffAbs = Math.abs(newHeading - crrHeading);
        double diff360Abs = 360 - diffAbs;
        double direction = newHeading > crrHeading ? 1.0 : -1.0;
        double inverted = diffAbs < diff360Abs ? 1.0 : -1.0;

        turnDeg = inverted > 0 ? diffAbs : diff360Abs;
        turnDeg *= direction * inverted;

        turn(turnDeg);
    }

    void turn(double turnDeg) {

        double direction = turnDeg > 0 ? 1.0 : -1.0;
        double crrError = turnDeg * direction;
        double startHeading = robot.modernRoboticsI2cGyro.getHeading();
        double endHeading = startHeading + turnDeg;

        while (crrError > 5) {

            leftDriveControl = -0.2 * direction; //power to motors is proportional with the speed
            rightDriveControl = 0.2 * direction;

            setDrives();
            waitMillis(5);

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

        double direction = distance > 0 ? 1.0 : -1.0;

        Animator moveAnimator = new Animator();
        moveAnimator.configRamp(333, 444);
        moveAnimator.configSpeed(0.1, 0.88, 0.2, 5);
        moveAnimator.start(0, Math.abs(distance));

        moveAnimator.advanceStepNoPos();
        double nextSpeed = moveAnimator.getSpeed();

        while (nextSpeed > 0) {

            leftDriveControl = nextSpeed * direction; //power to motors is proportional with the speed
            rightDriveControl = nextSpeed * direction;

            setDrives();
            sleep(5);

            moveAnimator.advanceStepNoPos();
            nextSpeed = moveAnimator.getSpeed();
        }
        stopRobot();
    }

    // ************************** ARM  DRIVE SERVOS HELPER FUNCTIONS  ****************************//

    void moveArm(double newBase, double newElbow) {

        if (!armEnabled) {
            return;
        }

        double baseStart = baseControl;
        double elbowStart = elbowControl;

        double stepSize = 0.01;
        double stepCount = Math.abs(baseStart - newBase) / stepSize;
        stepCount = Math.max(Math.abs(elbowStart - newElbow) / stepSize, stepCount);
        double elbowStepSize = (newElbow - baseStart) / stepCount;
        double baseStepSize = (newBase - elbowStart) / stepCount;

        for (int i = 0; i < (int) stepCount; i++) {

            double baseCrr = baseStart + i * baseStepSize;
            double elbowCrr = elbowStart + i * elbowStepSize;

            baseControl = baseCrr;
            elbowControl = elbowCrr;
            setServos();
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
            double error = headingControl - headingCrr;
            double driveDirection = leftDriveControl > 0 ? 1.0 : -1.0;

            if (headingControl > 180 && headingCrr <= 180 && headingControl - headingCrr > 180) {
                headingCrr += 360;
            }
            if (headingControl <= 180 && headingCrr > 180 && headingCrr - headingControl > 180) {
                headingCrr -= 360;
            }
            error = headingCrr - headingControl;
            error = Range.clip(error, -45, 45); // if completely off trim down

            headingCorrection = error / 45 * 0.11; //TODO tune up the amount of correction
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

    void setServos() {

        double minLeftClaw = Math.min(Math.min(clasZero[0], clawClosed[0]), clawOpen[0]);
        double maxLeftClaw = Math.max(Math.max(clasZero[0], clawClosed[0]), clawOpen[0]);

        double minRightClaw = Math.min(Math.min(clasZero[1], clawClosed[1]), clawOpen[1]);
        double maxRightClaw = Math.max(Math.max(clasZero[1], clawClosed[1]), clawOpen[1]);

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

    void checkAndStopAutonomous() {

        if (manualMode) {
            return;
        }

        if (stopTime(30)) {
            stopRobot();
            stop(); //stop the opMode
        }
    }

    void stopRobot() {
        leftDriveControl = 0;
        rightDriveControl = 0;
        setDrives();
    }

    void waitMillis(double millis) {

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

