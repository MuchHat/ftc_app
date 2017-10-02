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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file illustrates the concept of driving a path based on time.
 */

@TeleOp(name = "OpMode V6", group = "Gigi")
// @Disabled
public class Gigi_OpMode_V6 extends LinearOpMode {

    public Gigi_Hardware_V2 robot = new Gigi_Hardware_V2();
    public ElapsedTime runtime = new ElapsedTime();

    public Arm theArm = null;
    public Arm testArm = null;

    public double turretControl = 0;
    public double baseControl = 0;
    public double elbowControl = 0;
    public double wristControl = 0;
    public double clawControlL = 0;
    public double clawControlR = 0;

    public double xControl = 0;
    public double yControl = 0;
    public double zControl = 0;

    public double lControl = 0;
    public double rControl = 0;

    public double xControlLast = 0;
    public double yControlLast = 0;
    public double zControlLast = 0;

    public double lControlLast = 0;
    public double rControlLast = 0;

    public double baseControlLast = 0;
    public double elbowControlLast = 0;
    public double wristControlLast = 0;
    public double turretControlLast = 0;

    boolean useAxisControl = false;
    boolean useDriveControl = false;

    double servoDefaultSpeed = 0.33 / 1000; // 0.3 servo angle per sec
    double axisDefaultSpeed = 33 / 1000; // 33 mm per sec
    double driveDefaultSpeed = 0.1; // ??
    double turnDefaultSpeed = 0.01; // ??
    double kDrive = 0.33; // ??

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        theArm = new Arm();
        theArm.init();
        testArm = new Arm();
        testArm.init();

        testArm.setZeroXYZ();

        robot._turret.setPosition(testArm.getTurretServo());
        robot._base.setPosition(testArm.getBaseServo());
        robot._elbow.setPosition(testArm.getElbowServo());
        robot._wrist.setPosition(testArm.getWristServo());
        robot._leftClaw.setPosition(testArm.getLeftClawServo());
        robot._rightClaw.setPosition(testArm.getRightClawServo());

        xControl = testArm.getX();
        yControl = testArm.getY();
        zControl = testArm.getZ();

        turretControl = robot._turret.getPosition();
        baseControl = robot._base.getPosition();
        elbowControl = robot._elbow.getPosition();
        wristControl = robot._wrist.getPosition();
        clawControlL = robot._leftClaw.getPosition();
        clawControlR = robot._rightClaw.getPosition();

        lControl = 0;
        rControl = 0;

        xControlLast = xControl;
        yControlLast = yControl;
        zControlLast = zControl;

        lControlLast = lControl;
        rControlLast = rControl;

        baseControlLast = baseControl;
        elbowControlLast = elbowControl;
        wristControlLast = wristControl;
        turretControlLast = turretControl;

        waitForStart();

        while (opModeIsActive()) {

            double crrLoopTime = runtime.nanoseconds() / 1000000; // coverted to millis
            runtime.reset();

            turretControl = robot._turret.getPosition();
            baseControl = robot._base.getPosition();
            elbowControl = robot._elbow.getPosition();
            wristControl = robot._wrist.getPosition();
            clawControlL = robot._leftClaw.getPosition();
            clawControlR = robot._rightClaw.getPosition();

            testArm.setServos(turretControl, baseControl, elbowControl);

            telemetry.addData("loopTime", "{%.3fms}",
                    crrLoopTime);

            telemetry.addData("servos control->", "{%.3f  %.3f  %.3f}",
                    turretControl,
                    baseControl,
                    elbowControl);

            telemetry.addData("servos control->", "{%.3f} {%.3f %.3f}",
                    wristControl,
                    clawControlR,
                    clawControlL);

            telemetry.addData("drives control->", "{%.3f %.3f}",
                    lControl,
                    rControl);

            telemetry.addData("xyz control->", "{%.0fmm  %.0fmm  %.0fmm}",
                    xControl,
                    yControl,
                    zControl);

            telemetry.addData("xyz ->", "{%.0fmm  %.0fmm  %.0fmm}",
                    theArm.getX(),
                    theArm.getY(),
                    theArm.getZ());

            telemetry.addData("rtp->", "{%.0fmm  %.0fg  %.0fg, %.0fg}",
                    theArm.getR(),
                    theArm.getTeta() / Math.PI * 180,
                    theArm.getPhi() / Math.PI * 180,
                    theArm.getA2() / Math.PI * 180);

            telemetry.addData("tbew->", "{%.3f  %.3f  %.3f, %.3f}",
                    theArm.getTurretServo(),
                    theArm.getBaseServo(),
                    theArm.getElbowServo(),
                    theArm.getWristServo());

            telemetry.addData("test servos->", "{%.3f  %.3f  %.3f, %.3f}",
                    testArm.getTurretServo(),
                    testArm.getBaseServo(),
                    testArm.getElbowServo(),
                    testArm.getWristServo());

            telemetry.addData("test xyz ->", "{%.0fmm  %.0fmm  %.0fmm}",
                    testArm.getX(),
                    testArm.getY(),
                    testArm.getZ());

            telemetry.addData("test rtp->", "{%.0fmm  %.0fg  %.0fg, %.0fg}",
                    testArm.getR(),
                    testArm.getTeta() / Math.PI * 180,
                    testArm.getPhi() / Math.PI * 180,
                    testArm.getA2() / Math.PI * 180);

            telemetry.update();

            /**************************** START  OF USE AXIS CONTROL*****************************/

            if (!useAxisControl) {

                theArm.setServos(turretControl, baseControl, elbowControl);

                // control: BASE
                if (gamepad1.left_stick_y != 0) {
                    baseControl += gamepad1.left_stick_y * servoDefaultSpeed * crrLoopTime;
                    angleSetServos();
                }
                // control: WRIST
                if (gamepad1.left_stick_x != 0) {
                    wristControl += gamepad1.left_stick_x * servoDefaultSpeed * crrLoopTime;
                    angleSetServos();
                }
                // control: ELBOW
                if (gamepad1.right_stick_y != 0) {
                    elbowControl += gamepad1.right_stick_y * servoDefaultSpeed * crrLoopTime;
                    angleSetServos();

                }
                // control: TURRET
                if (gamepad1.right_stick_x != 0) {
                    turretControl += gamepad1.right_stick_x * servoDefaultSpeed * crrLoopTime;
                    angleSetServos();

                }
                // control: CLAW CLOSE
                if (gamepad1.left_trigger != 0) {
                    clawControlL -= gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                    clawControlR += gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                    angleSetServos();
                }
                // control: CLAW OPEN
                if (gamepad1.right_trigger != 0) {
                    clawControlL += gamepad1.right_trigger * servoDefaultSpeed * crrLoopTime;
                    clawControlR -= gamepad1.right_trigger * servoDefaultSpeed * crrLoopTime;
                    angleSetServos();
                }
                // control:
                if (gamepad1.back) {
                    testArm.setZeroXYZ();

                    robot._turret.setPosition(testArm.getTurretServo());
                    robot._base.setPosition(testArm.getBaseServo());
                    robot._elbow.setPosition(testArm.getElbowServo());
                    robot._wrist.setPosition(testArm.getWristServo());
                    robot._leftClaw.setPosition(testArm.getLeftClawServo());
                    robot._rightClaw.setPosition(testArm.getRightClawServo());
                }
                // control:
                if (gamepad1.dpad_up) {
                }
                // control:
                if (gamepad1.dpad_down) {
                }
                // control:
                if (gamepad1.dpad_right) {
                }
                // control:
                if (gamepad1.dpad_left) {
                }
                // control:
                if (gamepad1.right_bumper) {
                }
                // control:
                if (gamepad1.left_bumper) {
                }
                if (gamepad1.b) {
                    useDriveControl = false;
                    useAxisControl = true;
                }
                // control:
                if (gamepad1.a) {
                    useAxisControl = false;
                    useDriveControl = false;
                }
                // control:
                if (gamepad1.x) {
                    useDriveControl = true;
                }
                // control:
                if (gamepad1.y) {
                }
            }
            /**************************** START  OF USE AXIS CONTROL*****************************/

            if (useAxisControl) {

                theArm.setXYZ(xControl, yControl, zControl);

                // control: TURRET
                if (gamepad1.left_stick_y != 0) {
                    zControl += gamepad1.left_stick_y * axisDefaultSpeed * crrLoopTime;// * axisDefaultSpeed * crrLoopTime;
                    xyzSetServos();
                }
                // control: BASE
                if (gamepad1.right_stick_y != 0) {
                    yControl += -gamepad1.right_stick_y * axisDefaultSpeed * crrLoopTime;// * axisDefaultSpeed * crrLoopTime;
                    xyzSetServos();
                }
                // control: WRIST
                if (gamepad1.right_stick_x != 0) {
                    xControl += -gamepad1.right_stick_x * axisDefaultSpeed * crrLoopTime;// * axisDefaultSpeed * crrLoopTime;
                    xyzSetServos();
                }
                // control: ELBOW
                if (gamepad1.left_stick_x != 0) {
                    clawControlR += gamepad1.left_stick_x * axisDefaultSpeed * crrLoopTime;
                    clawControlL -= gamepad1.left_stick_x * axisDefaultSpeed * crrLoopTime;
                    xyzSetServos();

                }
                if (gamepad1.back) {
                    testArm.setZeroXYZ();

                    robot._turret.setPosition(testArm.getTurretServo());
                    robot._base.setPosition(testArm.getBaseServo());
                    robot._elbow.setPosition(testArm.getElbowServo());
                    robot._wrist.setPosition(testArm.getWristServo());
                    robot._leftClaw.setPosition(testArm.getLeftClawServo());
                    robot._rightClaw.setPosition(testArm.getRightClawServo());

                    xControl = testArm.getX();
                    yControl = testArm.getY();
                    zControl = testArm.getZ();
                }
                if (gamepad1.b) {
                    useDriveControl = false;
                    useAxisControl = true;
                }
                // control:
                if (gamepad1.a) {
                    useAxisControl = false;
                    useDriveControl = false;
                }
                // control:
                if (gamepad1.x) {
                    useDriveControl = true;
                }
            }

            /**************************** START  OF USE DRIVE CONTROL*****************************/

            if (useDriveControl) {

                theArm.setXYZ(xControl, yControl, zControl);

                // control: TURRET
                if (gamepad1.left_stick_y != 0) {
                    lControl += gamepad1.left_stick_y * driveDefaultSpeed * crrLoopTime;// * axisDefaultSpeed * crrLoopTime;
                    rControl += gamepad1.left_stick_y * driveDefaultSpeed * crrLoopTime;// * axisDefaultSpeed * crrLoopTime;
                    driveSetPower();
                }
                // control: BASE
                if (gamepad1.right_stick_x != 0) {
                    lControl -= gamepad1.right_stick_x * turnDefaultSpeed * crrLoopTime;// * axisDefaultSpeed * crrLoopTime;
                    rControl += gamepad1.right_stick_x * turnDefaultSpeed * crrLoopTime;// * axisDefaultSpeed * crrLoopTime;
                    driveSetPower();
                }
                // control: WRIST
                if (gamepad1.right_stick_y != 0) {
                }
                // control: ELBOW
                if (gamepad1.left_stick_x != 0) {
                }
                if (gamepad1.back) {
                    lControl = 0;// * axisDefaultSpeed * crrLoopTime;
                    rControl = 0;// * axisDefaultSpeed * crrLoopTime;
                    lControlLast = 0;
                    rControlLast = 0;
                    driveSetPower();
                }
                if (gamepad1.b) {
                    useDriveControl = false;
                    useAxisControl = true;
                }
                // control:
                if (gamepad1.a) {
                    useAxisControl = false;
                    useDriveControl = false;
                }
                // control:
                if (gamepad1.x) {
                    useDriveControl = true;
                }
            }
        }
    }

    public void angleSetServos() {

        clawControlL = Range.clip(clawControlL, theArm.leftClawAngle.minServo, theArm.leftClawAngle.maxServo);
        clawControlR = Range.clip(clawControlR, theArm.rightClawAngle.minServo, theArm.rightClawAngle.maxServo);

        robot._leftClaw.setPosition(clawControlL);
        robot._rightClaw.setPosition(clawControlR);

        elbowControl = Range.clip(elbowControl, theArm.elbowAngle.minServo, theArm.elbowAngle.maxServo);
        baseControl = Range.clip(baseControl, theArm.baseAngle.minServo, theArm.baseAngle.maxServo);
        wristControl = Range.clip(wristControl, theArm.wristAngle.minServo, theArm.wristAngle.maxServo);
        turretControl = Range.clip(turretControl, theArm.turretAngle.minServo, theArm.turretAngle.maxServo);

        //do a collision check
        testArm.setServos(turretControl, baseControl, elbowControl);
        if (!testArm.collisionCheck(false)) {

            // slow down if needed
            double maxServoStep = 0.1; // 6mm per axis and step
            double stepCount = 0;
            stepCount = Math.max(stepCount, Math.abs(elbowControl - elbowControlLast) / maxServoStep);
            stepCount = Math.max(stepCount, Math.abs(baseControl - baseControlLast) / maxServoStep);
            stepCount = Math.max(stepCount, Math.abs(wristControl - wristControlLast) / maxServoStep);
            stepCount = Math.max(stepCount, Math.abs(turretControl - turretControlLast) / maxServoStep);

            if (stepCount > 0) {
                for (int i = 0; i < stepCount; i++) {
                    double elbowControlStep = elbowControlLast + i* (elbowControl - elbowControlLast) / maxServoStep;
                    double baseControlStep = baseControlLast + i* (baseControl - baseControlLast) / maxServoStep;
                    double wristControlStep = wristControlLast + i* (wristControl - wristControlLast) / maxServoStep;
                    double turretControlStep = turretControlLast + i* (turretControl - turretControlLast) / maxServoStep;
                    //TODO - add colision detection?

                    // use the order for minimum colision
                    robot._elbow.setPosition(elbowControlStep);
                    robot._base.setPosition(baseControlStep);
                    robot._wrist.setPosition(wristControlStep);
                    robot._turret.setPosition(turretControlStep);
                    //TODO : add delay
                }
            }

            robot._elbow.setPosition(elbowControl);
            robot._base.setPosition(baseControl);
            robot._wrist.setPosition(wristControl);
            robot._turret.setPosition(turretControl);

            baseControlLast = baseControl;
            elbowControlLast = elbowControl;
            wristControlLast = wristControl;
            turretControlLast = turretControl;
        }
    }

    public void xyzSetServos() {

        theArm.setXYZ(xControl, yControl, zControl);

        //slow down the move here if needed, also use for collision correction
        double maxStep = 6; // 6mm per axis and step
        double stepCount = 0;
        stepCount = Math.max(stepCount, Math.abs(xControl - xControlLast) / maxStep);
        stepCount = Math.max(stepCount, Math.abs(yControl - yControlLast) / maxStep);
        stepCount = Math.max(stepCount, Math.abs(zControl - zControlLast) / maxStep);

        if (stepCount > 0) {
            Arm stepArm = new Arm();

            for (int i = 0; i < stepCount; i++) {
                stepArm.setXYZ(xControlLast + ((xControl - xControlLast) / stepCount) * i,
                        yControlLast + ((yControl - yControlLast) / stepCount) * i,
                        zControlLast + ((zControl - zControlLast) / stepCount) * i);

                // use the order for minimum colision
                robot._elbow.setPosition(stepArm.getTurretServo());
                robot._base.setPosition(stepArm.getBaseServo());
                robot._turret.setPosition(stepArm.getTurretServo());
                robot._wrist.setPosition(stepArm.getWristServo());
                robot._leftClaw.setPosition(stepArm.getLeftClawServo());
                robot._rightClaw.setPosition(stepArm.getRightClawServo());
                //TODO : add delay
            }
        }

        robot._elbow.setPosition(theArm.getTurretServo());
        robot._base.setPosition(theArm.getBaseServo());
        robot._turret.setPosition(theArm.getTurretServo());
        robot._wrist.setPosition(theArm.getWristServo());
        robot._leftClaw.setPosition(theArm.getLeftClawServo());
        robot._rightClaw.setPosition(theArm.getRightClawServo());

        xControlLast = xControl;
        yControlLast = yControl;
        zControlLast = zControl;
    }

    public void driveSetPower() {

        double kCrr = kDrive;

        double lErr = Range.clip(lControl - lControlLast, 0, 1);
        double rErr = Range.clip(rControl - rControlLast, 0, 1);

        //TODO - adjust below
        if (lErr < 0.15 && rErr < 0.15) {
            kCrr = 1;
        }

        robot.leftDrive.setPower(lErr * kCrr);
        robot.rightDrive.setPower(lErr * kCrr);

        lControlLast = lControl;
        rControlLast = rControl;
    }
}

