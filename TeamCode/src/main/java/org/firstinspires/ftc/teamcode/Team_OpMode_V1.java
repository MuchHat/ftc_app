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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.R.attr.delay;

/**
 * This file illustrates the concept of driving a path based on time.
 */

@TeleOp(name = "Team V1", group = "Team")
// @Disabled
public class Team_OpMode_V1 extends LinearOpMode {

    public Gigi_Hardware_V2 robot = new Gigi_Hardware_V2();
    public ElapsedTime runtime = new ElapsedTime();

    public Arm theArm = null;

    public double turretControl = 0;
    public double baseControl = 0;
    public double elbowControl = 0;
    public double wristControl = 0;
    public double leftClawControl = 0;
    public double rightClawControl = 0;

    public double xControl = 0;
    public double yControl = 0;
    public double zControl = 0;

    public double lControl = 0;
    public double rControl = 0;

    public double lControlLast = 0;
    public double rControlLast = 0;

    public double baseControlLast = 0;
    public double elbowControlLast = 0;
    public double wristControlLast = 0;
    public double turretControlLast = 0;

    boolean doArmControl = false;
    boolean doDriveControl = false;

    double servoDefaultSpeed = 0.00033; // 0.33 servo angle per sec
    double driveDefaultSpeed = 0.44; // ??
    double turnDefaultSpeed = 0.22; // ??

    String currentPos = new String("none");
    String lastPos = new String("none");

    double posZero[] = {0.40, 0.41, 0.95, 0.85};
    double posFront[] = {0.40, 0.59, 0.36, 0.05};
    double posFrontUp[] = {0.40, 0.42, 0.58, 0.45 };

    double clawOpen[] = {0.76, 0.44};
    double clawClosed[] = {0.85, 0.34};
    double clawZero[] = {0.76, 0.44};

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        theArm = new Arm();
        theArm.init();

        theArm.turretAngle.Init(45, 90, 0.13, 0.41, 0.20, 0.56); // turret setup
        theArm.baseAngle.Init(45, 90, 0.95, 0.75, 0.05, 0.95); // //base setup
        theArm.elbowAngle.Init(45, 90, 0.32, 0.71, 0.05, 0.95); //elbow setup
        theArm.wristAngle.Init(45, 90, 0.89, 0.58, 0.05, 0.95);  //wrist setup
        theArm.leftClawAngle.Init(0, 45, 0.55, 0.22, 0.05, 0.95); // left claw setup
        theArm.rightClawAngle.Init(0, 45, 0.44, 0.76, 0.31, 0.95); // right claw setup

        robot._turret.setPosition(posZero[0]);
        robot._base.setPosition(posZero[1]);
        robot._elbow.setPosition(posZero[2]);
        robot._wrist.setPosition(posZero[3]);
        robot._leftClaw.setPosition(clawZero[0]);
        robot._rightClaw.setPosition(clawZero[1]);
        currentPos = "zero";

        lControl = 0;
        rControl = 0;

        lControlLast = lControl;
        rControlLast = rControl;
        baseControlLast = baseControl;
        elbowControlLast = elbowControl;
        wristControlLast = wristControl;
        turretControlLast = turretControl;

        waitForStart();

        while (opModeIsActive()) {

            double crrLoopTime = runtime.nanoseconds() / 1000000; // covert to millis
            runtime.reset();

            turretControl = robot._turret.getPosition();
            baseControl = robot._base.getPosition();
            elbowControl = robot._elbow.getPosition();
            wristControl = robot._wrist.getPosition();
            leftClawControl = robot._leftClaw.getPosition();
            rightClawControl = robot._rightClaw.getPosition();

            theArm.turretAngle.setServo(turretControl);
            theArm.baseAngle.setServo(baseControl);
            theArm.elbowAngle.setServo(elbowControl);
            theArm.wristAngle.setServo(wristControl);
            theArm.leftClawAngle.setServo(leftClawControl);
            theArm.rightClawAngle.setServo(rightClawControl);

            String controlMode = new String("->none");
            if (doDriveControl) controlMode = "->drive";
            if (doArmControl) controlMode = "->arm";

            telemetry.addData("mode", controlMode);
            telemetry.addData("position", currentPos);
            telemetry.addData("Turret->", "{%.0f%% %.0fdeg}",
                    turretControl * 100, theArm.turretAngle.getPI() / Math.PI * 180);
            telemetry.addData("Base->", "{%.0f%% %.0fdeg}",
                    baseControl * 100, theArm.baseAngle.getPI() / Math.PI * 180);
            telemetry.addData("Elbow->", "{%.0f%% %.0fdeg}",
                    elbowControl * 100, theArm.elbowAngle.getPI() / Math.PI * 180);
            telemetry.addData("Wrist->", "{%.0f%% %.0fdeg}",
                    wristControl * 100, theArm.wristAngle.getPI() / Math.PI * 180);
            telemetry.addData("LeftClaw->", "{%.0f%% %.0fdeg}",
                    leftClawControl * 100, theArm.leftClawAngle.getPI() / Math.PI * 180);
            telemetry.addData("RightClaw->", "{%.0f%% %.0fdeg}",
                    rightClawControl * 100, theArm.rightClawAngle.getPI() / Math.PI * 180);
            telemetry.addData("loopTime", "{%.3fms}", crrLoopTime);
            telemetry.update();

            // control:
            if (gamepad1.a) {
                doDriveControl = true;
                doArmControl = false;
            }
            if (gamepad1.b) {
                doDriveControl = false;
                doArmControl = true;
            }

            /**************************** START  OF USE SERVOS CONTROL*****************************/

            if (doArmControl && !doDriveControl) {

                // control: BASE
                if (gamepad1.left_stick_y != 0) {
                    baseControl += gamepad1.left_stick_y * servoDefaultSpeed * crrLoopTime;
                    currentPos = "none";
                    setServos();
                }
                // control: WRIST
                if (gamepad1.left_stick_x != 0) {
                    wristControl += gamepad1.left_stick_x * servoDefaultSpeed * crrLoopTime;
                    currentPos = "none";
                    setServos();
                }
                // control: ELBOW
                if (gamepad1.right_stick_y != 0) {
                    elbowControl += gamepad1.right_stick_y * servoDefaultSpeed * crrLoopTime;
                    currentPos = "none";
                    setServos();
                }
                // control: TURRET
                if (gamepad1.right_stick_x != 0) {
                    turretControl += gamepad1.right_stick_x * servoDefaultSpeed * crrLoopTime;
                    currentPos = "none";
                    setServos();
                }
                // control: CLAW CLOSE
                if (gamepad1.left_trigger != 0) {
                    leftClawControl -= gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                    rightClawControl += gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }
                // control: CLAW OPEN
                if (gamepad1.right_trigger != 0) {
                    leftClawControl += gamepad1.right_trigger * servoDefaultSpeed * crrLoopTime;
                    rightClawControl -= gamepad1.right_trigger * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }
                // control:
                if (gamepad1.x) {
                    turretControl = posZero[0];
                    baseControl = posZero[1];
                    elbowControl = posZero[2];
                    wristControl = posZero[3];
                    leftClawControl = clawZero[0];
                    rightClawControl = clawZero[1];
                    currentPos = "zero";
                    setServos();
                }
                // control:
                if (gamepad1.dpad_down) {
                    turretControl = posFront[0];
                    baseControl = posFront[1];
                    elbowControl = posFront[2];
                    wristControl = posFront[3];
                    currentPos = "front";
                    setServos();
                }
                // control:
                if (gamepad1.dpad_up) {
                    turretControl = posFrontUp[0];
                    baseControl = posFrontUp[1];
                    elbowControl = posFrontUp[2];
                    wristControl = posFrontUp[3];
                    currentPos = "front up";
                    setServos();
                }
                // control:
                if (gamepad1.left_bumper) {
                    leftClawControl = clawOpen[0];
                    rightClawControl = clawOpen[1];
                    setServos();
                }
                // control
                if (gamepad1.right_bumper) {
                    leftClawControl = clawClosed[0];
                    rightClawControl = clawClosed[1];
                    setServos();
                }
            }
            /**************************** START  OF USE DRIVE CONTROL*****************************/

            if (doDriveControl && !doArmControl) {
                {
                    double xInput = gamepad1.left_stick_x;
                    double yInput = -gamepad1.right_stick_y;

                    if (Math.abs(xInput) < 0.15) xInput = 0; //acts as brake
                    if (Math.abs(yInput) < 0.15) yInput = 0; //acts as brake

                    lControl = yInput * driveDefaultSpeed;
                    rControl = yInput * driveDefaultSpeed;

                    lControl -= xInput * turnDefaultSpeed;
                    rControl += xInput * turnDefaultSpeed;
                    setDrives();
                }
                if (gamepad1.back) {
                    lControl = 0;//
                    rControl = 0;//
                    setDrives();
                }
            }
        }
    }

    public void setServos() {

        boolean doInStages = false;

        // determine do in stages based on the base angle before and after
        // posZero[1] is base and pozFront[1] is base
        if (Math.abs(baseControl - posZero[1]) < 0.2 ||
                Math.abs(baseControlLast - posZero[1]) < 0.2) {
            doInStages = true;
        }
        //TODO
        doInStages = false;

        elbowControl = Range.clip(elbowControl, theArm.elbowAngle.minServo, theArm.elbowAngle.maxServo);
        baseControl = Range.clip(baseControl, theArm.baseAngle.minServo, theArm.baseAngle.maxServo);
        wristControl = Range.clip(wristControl, theArm.wristAngle.minServo, theArm.wristAngle.maxServo);
        turretControl = Range.clip(turretControl, theArm.turretAngle.minServo, theArm.turretAngle.maxServo);
        leftClawControl = Range.clip(leftClawControl, theArm.leftClawAngle.minServo, theArm.leftClawAngle.maxServo);
        rightClawControl = Range.clip(rightClawControl, theArm.rightClawAngle.minServo, theArm.rightClawAngle.maxServo);

        // slow down if needed
        double maxServoStep = 0.004; // 0.1 per servo and step
        double stepCount = 0;
        stepCount = Math.max(stepCount, Math.abs(elbowControl - elbowControlLast) / maxServoStep);
        stepCount = Math.max(stepCount, Math.abs(baseControl - baseControlLast) / maxServoStep);
        stepCount = Math.max(stepCount, Math.abs(wristControl - wristControlLast) / maxServoStep);
        stepCount = Math.max(stepCount, Math.abs(turretControl - turretControlLast) / maxServoStep);

        if (stepCount > 0) {
            // move the elbow first to avoid hitting the robot

            ElapsedTime stepElapsedTime = new ElapsedTime();
            stepElapsedTime.reset();

            if (doInStages) {
                for (int i = 0; i < stepCount; i++) {

                    double elbowControlStep = elbowControlLast + i * (elbowControl - elbowControlLast) / stepCount;
                    elbowControlStep = Range.clip(elbowControlStep, theArm.elbowAngle.minServo, theArm.elbowAngle.maxServo);
                    while (stepElapsedTime.milliseconds() < 3 * (i + 1)) {
                        idle();
                    }
                    robot._elbow.setPosition(elbowControlStep);
                }
            }
            stepElapsedTime.reset();
            for (int i = 0; i < stepCount; i++) {

                double baseControlStep = baseControlLast + i * (baseControl - baseControlLast) / stepCount;
                double wristControlStep = wristControlLast + i * (wristControl - wristControlLast) / stepCount;
                double turretControlStep = turretControlLast + i * (turretControl - turretControlLast) / stepCount;
                double elbowControlStep = elbowControlLast + i * (elbowControl - elbowControlLast) / stepCount;

                elbowControlStep = Range.clip(elbowControlStep, theArm.elbowAngle.minServo, theArm.elbowAngle.maxServo);
                baseControlStep = Range.clip(baseControlStep, theArm.baseAngle.minServo, theArm.baseAngle.maxServo);
                wristControlStep = Range.clip(wristControlStep, theArm.wristAngle.minServo, theArm.wristAngle.maxServo);
                turretControlStep = Range.clip(turretControlStep, theArm.turretAngle.minServo, theArm.turretAngle.maxServo);

                while (stepElapsedTime.milliseconds() < 3 * (i + 1)) {
                    idle();
                }
                if (!doInStages) robot._elbow.setPosition(elbowControlStep);
                robot._base.setPosition(baseControlStep);
                robot._wrist.setPosition(wristControlStep);
                robot._turret.setPosition(turretControlStep);
            }
        }

        robot._elbow.setPosition(elbowControl);
        robot._base.setPosition(baseControl);
        robot._wrist.setPosition(wristControl);
        robot._turret.setPosition(turretControl);
        robot._leftClaw.setPosition(leftClawControl);
        robot._rightClaw.setPosition(rightClawControl);

        baseControlLast = baseControl;
        elbowControlLast = elbowControl;
        wristControlLast = wristControl;
        turretControlLast = turretControl;
    }

    public void setDrives() {

        if (lControl >= 0) {
            robot.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftDrive.setPower(lControl);
        } else if (lControl < 0) {
            robot.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.leftDrive.setPower(-lControl);
        }
        if (rControl >= 0) {
            robot.rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.rightDrive.setPower(rControl);
        } else if (rControl < 0) {
            robot.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive.setPower(-rControl);
        }

        lControlLast = lControl;
        rControlLast = rControl;
    }
}

