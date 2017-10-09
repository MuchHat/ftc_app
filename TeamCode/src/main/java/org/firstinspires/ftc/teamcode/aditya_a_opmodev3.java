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

@TeleOp(name = "yo", group = "Team")
// @Disabled
public class aditya_a_opmodev3 extends LinearOpMode {

    public Team_Hardware_V2 robot = new Team_Hardware_V2();

    public ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    public IntegratingGyroscope gyro;

    public ElapsedTime runtimeLoop = new ElapsedTime();

    public double leftControl = 0;
    public double rightControl = 0;
    public double headingControl = 0;
    public double liftControl = 0;
    public double leftClawControl = 0;
    public double rightClawControl = 0;

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
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

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
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();

        leftControl = 0;
        rightControl = 0;
        liftControl = 0;
        leftClawControl = 0.5; // TODO
        rightClawControl = 0.5; // TODO
        headingControl = modernRoboticsI2cGyro.getHeading();

        gameStartHeading = headingControl;

        setDrives();
        setServos();

        telemetry.addData("crrHeading->", modernRoboticsI2cGyro.getHeading());
        telemetry.addData("lastHeading->", headingControl);
        telemetry.addData("gameStartHeading->", "{%.0fdeg}", gameStartHeading);
        telemetry.addData("LeftDrive->", "{%.0f%%}", leftControl * 100);
        telemetry.addData("RightDrive->", "{%.0f%%}", rightControl * 100);
        telemetry.addData("Lift->", "{%.0f%%}", liftControl * 100);
        telemetry.addData("LeftClaw->", "{%.0f%%}", leftClawControl * 100);
        telemetry.addData("RightClaw->", "{%.0f%%}", rightClawControl * 100);
        telemetry.update();

        waitForStart();
        runtimeLoop.reset();

        while (opModeIsActive()) {

            double crrLoopTime = runtimeLoop.nanoseconds() / 1000000; // covert to millis
            runtimeLoop.reset();

            telemetry.addData("crrHeading->", modernRoboticsI2cGyro.getHeading());
            telemetry.addData("lastHeading->", headingControl);
            telemetry.addData("gameStartHeading->", "{%.0fdeg}", gameStartHeading);
            telemetry.addData("LeftDrive->", "{%.0f%%}", leftControl * 100);
            telemetry.addData("RightDrive->", "{%.0f%%}", rightControl * 100);
            telemetry.addData("Lift->", "{%.0f%%}", liftControl * 100);
            telemetry.addData("LeftClaw->", "{%.0f%%}", leftClawControl * 100);
            telemetry.addData("RightClaw->", "{%.0f%%}", rightClawControl * 100);
            telemetry.update();

            // control: DRIVES
            {
                double xInput = 0;
                double yInput = 0;

                if (Math.abs(gamepad1.left_stick_x) > 0.15) xInput = gamepad1.left_stick_x;
                if (Math.abs(gamepad1.left_stick_y) > 0.15) yInput = gamepad1.left_stick_y;

                leftControl = yInput * driveDefaultSpeed;
                rightControl = yInput * driveDefaultSpeed;

                leftControl += xInput * turnDefaultSpeed;
                rightControl -= xInput * turnDefaultSpeed;

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
            if (gamepad1.b){
                leftControl = 0.1;
                rightControl = -0.1;
                setDrives();
                waitMillis(5);
                leftControl = 0;
                rightControl = 0;

            }
            if (gamepad1.x){
                leftControl = -0.1;
                rightControl = 0.1;
                setDrives();
                waitMillis(5);
                leftControl = 0;
                rightControl = 0;

            }
            // control: CLAW PREDEF OPEN
            if (gamepad1.y) {
                int i = 0;
                while(i<50)
                {
                    leftControl = 0.1;
                    rightControl = 0.1;
                    setDrives();
                    i++;
                    waitMillis(5);
                }
            }

            if (gamepad1.a){
                leftControl = -0.1;
                setDrives();
                waitMillis(1);
                rightControl = -0.1;
                setDrives();
                waitMillis(5);
                rightControl = -0.3;
                setDrives();
                waitMillis(10);
                rightControl = -0.5;
                setDrives();
                waitMillis(15);
                rightControl = 0;
            }
        }
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

        double startHeading = modernRoboticsI2cGyro.getHeading();
        double virtualStartHeading = startHeading + 360;
        double virtualEndHeading = virtualStartHeading + turnDeg;
        double direction = turnDeg > 0 ? 1.0 : -1.0;

        double error = Math.abs(virtualEndHeading - virtualStartHeading);
        double iterations = 0;

        while (error > 5 && iterations < 999) { //TODO
            double turnPower = 0.2; //TODO

            if (error < 15) {
                turnPower *= error / 15;
                turnPower = Range.clip(turnPower, 0.05, 0.2); //TODO
            }

            leftControl = turnPower * direction;
            rightControl = -turnPower * direction;
            setDrives();

            waitMillis( 5 ); //TODO

            double crrHeading = modernRoboticsI2cGyro.getHeading();
            double virtualCrrHeading = crrHeading + 360;

            if (startHeading >= 180 && crrHeading <= 180 && direction > 0) {
                virtualCrrHeading += 360;
            }
            if (startHeading < 180 && crrHeading > 180 && direction < 0) {
                virtualCrrHeading -= 360;
            }

            error = Math.abs(virtualEndHeading - virtualCrrHeading);
            iterations++;
        }
        leftControl = 0;
        rightControl = 0;
        setDrives();

        headingControl = modernRoboticsI2cGyro.getHeading();
    }

    public void setDrives() {

        leftControl = Range.clip(leftControl, -0.66, 0.66); //TODO max max power
        rightControl = Range.clip(rightControl, -0.66, 0.66); //TODO max max power
        liftControl = Range.clip(liftControl, -0.66, 0.66); //TODO max max power

        if (leftControl >= 0) {
            robot.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftDrive.setPower(leftControl);
        }
        if (leftControl < 0) {
            robot.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.leftDrive.setPower(-leftControl);
        }
        if (rightControl >= 0) {
            robot.rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.rightDrive.setPower(rightControl);
        }
        if (rightControl < 0) {
            robot.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive.setPower(-rightControl);
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

    public void setServos() {

        leftClawControl = Range.clip(leftClawControl, 0.05, 0.95); //TODO
        rightClawControl = Range.clip(rightClawControl, 0.05, 0.95); //TODO

        robot.leftClaw.setPosition(leftClawControl);
        robot.rightClaw.setPosition(rightClawControl);
    }

    void waitMillis( double millis ){
        ElapsedTime runtimeWait = new ElapsedTime();

        runtimeWait.reset();

        while (runtimeWait.milliseconds() < millis) {
            idle();
        }
    }
}

