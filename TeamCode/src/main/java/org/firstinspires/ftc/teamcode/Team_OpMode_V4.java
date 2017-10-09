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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file illustrates the concept of driving a path based on time.
 */

@TeleOp(name = "Team V4", group = "Team")
// @Disabled
public class Team_OpMode_V4 extends LinearOpMode {

    public Team_Hardware_V2 robot = new Team_Hardware_V2();

    public ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    public IntegratingGyroscope gyro;

    public ElapsedTime runtimeLoop = new ElapsedTime();

    public double leftDriveControl = 0;
    public double rightDriveControl = 0;
    public double headingControl = 0;
    public double liftControl = 0;
    public double leftClawControl = 0;
    public double rightClawControl = 0;

    public double baseControl = 0;
    public double elbowControl = 0;

    Boolean armEnabled = false;

    Boolean manualMode = true;
    Boolean blueTeam = true;
    Boolean rightField = true;

    double driveDefaultSpeed = 0.44; // TODO
    double turnDefaultSpeed = 0.22; // TODO
    double liftDefaultSpeed = 0.22; // TODO
    double servoDefaultSpeed = 0.00033; // TODO

    double gameStartHeading = 0;

    double clawOpen[] = {0.76, 0.44};
    double clawClosed[] = {0.85, 0.34};

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = modernRoboticsI2cGyro;

        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        runtimeLoop.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(runtimeLoop.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }
        telemetry.log().clear();

        while (!isStopRequested() && !gamepad1.start) {

            String mode = manualMode ? "manual" : "autonomous";
            String field = rightField ? "RIGHT" : "LEFT";
            String team = blueTeam ? "BLUE" : "LEFT";

            telemetry.log().add("Gyro calibrated");
            telemetry.log().add("Select mode");
            telemetry.log().add("(A) for manual, (B) for autonomous: " + mode);
            telemetry.log().add("(X) for BLUE tem, (Y) for RED team: " + team);
            telemetry.log().add("(R bumper) for RIGHT field, (L bumper) for LEFT field: ", field);
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
        runtimeLoop.reset();

        while (opModeIsActive()) {

            //********************************* START LOOP *****************************************

            double crrLoopTime = runtimeLoop.nanoseconds() / 1000000; // covert to millis
            runtimeLoop.reset();

            updateTelemetry();

            //********************************* MANUAL MODE *****************************************

            if (manualMode) {

                // control: DRIVES
                {
                    double xInput = 0;
                    double yInput = 0;

                    if (Math.abs(gamepad1.left_stick_x) > 0.15) xInput = gamepad1.left_stick_x;
                    if (Math.abs(gamepad1.left_stick_y) > 0.15) yInput = -gamepad1.left_stick_y;

                    leftDriveControl = yInput * driveDefaultSpeed;
                    rightDriveControl = yInput * driveDefaultSpeed;

                    leftDriveControl += xInput * turnDefaultSpeed;
                    rightDriveControl -= xInput * turnDefaultSpeed;

                    setDrives();

                    if (xInput != 0) headingControl = modernRoboticsI2cGyro.getHeading();
                }

                // control: LIFT
                {
                    double liftInput = 0;

                    if (Math.abs(gamepad1.right_stick_y) > 0.15) liftInput = gamepad1.right_stick_y;

                    liftControl = liftInput * liftDefaultSpeed;

                    setDrives();
                }

                // control: TURNS 90
                if (gamepad1.dpad_right) {
                    doTurn(90);
                }

                // control: TURNS 90
                if (gamepad1.dpad_left) {
                    doTurn(-90);
                }

                // control: TURNS 90
                if (gamepad1.dpad_left) {
                    doTurn(180);
                }

                // control: TURNS 90
                if (gamepad1.start) {
                    turnToHeading(gameStartHeading);
                }

                // control: CLAW OPEN
                if (gamepad1.left_trigger != 0) {
                    leftClawControl -= gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                    rightClawControl += gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }

                // control: CLAW CLOSE
                if (gamepad1.right_trigger != 0) {
                    leftClawControl += gamepad1.right_trigger * servoDefaultSpeed * crrLoopTime;
                    rightClawControl -= gamepad1.right_trigger * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }
                // control: CLAW PREDEF OPEN
                if (gamepad1.left_bumper) {
                    leftClawControl = clawOpen[0];
                    rightClawControl = clawOpen[1];
                    setServos();
                }
                // control: CLAW PREDEF CLOSE
                if (gamepad1.right_bumper) {
                    leftClawControl = clawClosed[0];
                    rightClawControl = clawClosed[1];
                    setServos();
                }
            }

            //********************************* AUTO MODE *****************************************

            if (!manualMode) {
                runAutonomous();
            }

            //********************************* END LOOP *****************************************
        }
    }

    // ************************** Helper Functions ***********************************************//

    public void runAutonomous() {

    }

    // ************************** Helper Functions ***********************************************//

    public void updateTelemetry() {
        telemetry.addData("currHeading->", "{%.0fdeg}", (double) modernRoboticsI2cGyro.getHeading());
        telemetry.addData("lastHeading->", "{%.0fdeg}", (double) headingControl);
        telemetry.addData("gameStartHeading->", "{%.0fdeg}", gameStartHeading);
        telemetry.addData("LeftDrive->", "{%.0f%%}", leftDriveControl * 100);
        telemetry.addData("RightDrive->", "{%.0f%%}", rightDriveControl * 100);
        telemetry.addData("Lift->", "{%.0f%%}", liftControl * 100);
        telemetry.addData("LeftClaw->", "{%.0f%%}", leftClawControl * 100);
        telemetry.addData("RightClaw->", "{%.0f%%}", rightClawControl * 100);
        telemetry.update();

    }


    public void turnToHeading(double newHeading) {

        newHeading %= 360;

        double crrHeading = modernRoboticsI2cGyro.getHeading();
        double turnDeg = newHeading - crrHeading;

        if (Math.abs(newHeading - crrHeading) > 360 - Math.abs(newHeading - crrHeading)) {
            double direction = newHeading > crrHeading ? 1.0 : -1.0;

            turnDeg = 360 - Math.abs(newHeading - crrHeading);
            turnDeg *= direction;
        }

        doTurn(turnDeg);
    }

    public void doTurn(double turnDeg) {

        turnDeg %= 360;
        if (turnDeg > 180) turnDeg = turnDeg - 360;
        else if (turnDeg < -180) turnDeg = turnDeg + 360;

        double startHeading = modernRoboticsI2cGyro.getHeading();
        double endHeading = startHeading + turnDeg;
        double direction = turnDeg > 0 ? 1.0 : -1.0;

        double error = Math.abs(endHeading - startHeading);
        double iterations = 0;

        while (error > 5 && iterations < 999) { //TODO
            double turnPower = 0.2; //TODO

            if (error < 15) {
                turnPower *= error / 15;
                turnPower = Range.clip(turnPower, 0.05, 0.2); //TODO
            }

            leftDriveControl = -turnPower * direction;
            rightDriveControl = turnPower * direction;
            setDrives();

            waitMillis(5); //TODO

            double crrHeading = modernRoboticsI2cGyro.getHeading();

            if (startHeading >= 180 && crrHeading <= 180 && direction > 0) {
                crrHeading += 360;
            }
            if (startHeading < 180 && crrHeading > 180 && direction < 0) {
                crrHeading -= 360;
            }

            error = Math.abs(endHeading - crrHeading);
            iterations++;
        }
        leftDriveControl = 0;
        rightDriveControl = 0;
        setDrives();

        headingControl = modernRoboticsI2cGyro.getHeading();
    }

    public void moveStraight(double mmDistance) {

        //time based
        for (int i = 0; i < mmDistance * 10; i++) {
            double error = i;
            if (i > mmDistance / 2) error = mmDistance - i;
            error = Range.clip(error, 20, 60); // 2mm to 6mm ramp

            leftDriveControl = 0.2 * error / 60;
            rightDriveControl = 0.2 * error / 60;
            setDrives();
            waitMillis(1);
        }

        leftDriveControl = 0;
        rightDriveControl = 0;
        setDrives();
    }

    void setDrives() {

        leftDriveControl = Range.clip(leftDriveControl, -0.66, 0.66); //TODO max max power
        rightDriveControl = Range.clip(rightDriveControl, -0.66, 0.66); //TODO max max power

        liftControl = Range.clip(liftControl, -0.66, 0.66); //TODO max max power

        if (liftControl >= 0 && robot.topSwitch.getState() == false) { // false means switch is pressed
            liftControl = 0;
        }
        if (liftControl < 0 && robot.bottomSwitch.getState() == false) { // false means switch is pressed
            liftControl = 0;
        }
        if (leftDriveControl >= 0) {
            robot.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftDrive.setPower(leftDriveControl);
        }
        if (leftDriveControl < 0) {
            robot.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.leftDrive.setPower(-leftDriveControl);
        }
        if (rightDriveControl >= 0) {
            robot.rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.rightDrive.setPower(rightDriveControl);
        }
        if (rightDriveControl < 0) {
            robot.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive.setPower(-rightDriveControl);
        }
        if (liftControl < 0) {
            robot.liftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.liftDrive.setPower(-liftControl);
        }
        if (liftControl >= 0) {
            robot.liftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.liftDrive.setPower(liftControl);
        }
    }

    void setServos() {

        leftClawControl = Range.clip(leftClawControl, 0.05, 0.95); //TODO
        rightClawControl = Range.clip(rightClawControl, 0.05, 0.95); //TODO

        robot.leftClaw.setPosition(leftClawControl);
        robot.rightClaw.setPosition(rightClawControl);
    }

    void moveArm(double newBase, double newElbow) {

        if (!armEnabled) {
            return;
        }

        newBase = Range.clip(newBase, 0.05, 0.95);
        newElbow = Range.clip(newElbow, 0.05, 0.95);

        double maxServoStep = 0.004; // 0.1 per servo and step
        double stepCount = 0;
        stepCount = Math.max(stepCount, Math.abs(newElbow - elbowControl) / maxServoStep);
        stepCount = Math.max(stepCount, Math.abs(newBase - baseControl) / maxServoStep);

        if (stepCount > 0) {
            // move the elbow first to avoid hitting the robot

            ElapsedTime stepElapsedTime = new ElapsedTime();
            stepElapsedTime.reset();

            for (int i = 0; i < stepCount; i++) {

                double baseControlStep = baseControl + i * (newBase - baseControl) / stepCount;
                double elbowControlStep = elbowControl + i * (newElbow - elbowControl) / stepCount;

                elbowControlStep = Range.clip(elbowControlStep, 0.05, 0.95);
                baseControlStep = Range.clip(baseControlStep, 0.05, 0.95);

                while (stepElapsedTime.milliseconds() < 3 * (i + 1)) {
                    idle();
                }
                robot.elbow.setPosition(elbowControlStep);
                robot.base.setPosition(baseControlStep);
            }
        }

        robot.elbow.setPosition(newElbow);
        robot.base.setPosition(newBase);

        baseControl = newBase;
        elbowControl = newElbow;

    }

    void waitMillis(double millis) {
        ElapsedTime runtimeWait = new ElapsedTime();

        runtimeWait.reset();

        while (runtimeWait.milliseconds() < millis) {
            idle();
        }
    }
}

