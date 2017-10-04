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

/**
 * This file illustrates the concept of driving a path based on time.
 */

@TeleOp(name = "OpMode V7", group = "Gigi")
// @Disabled
public class Gigi_OpMode_V8 extends LinearOpMode {

    public Gigi_Hardware_V2 robot = new Gigi_Hardware_V2();
    public ElapsedTime runtime = new ElapsedTime();

    public Arm theArm = null;
    public Arm testArm = null;

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

    double servoDefaultSpeed = 0.00033; // 0.33 servo angle per sec
    double axisDefaultSpeed = 0.44; // 1100 mm per sec
    double driveDefaultSpeed = 0.44; // ??
    double turnDefaultSpeed = 0.22; // ??
    double kDrive = 1; // ??

    boolean extendedLogging = true;

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
        leftClawControl = robot._leftClaw.getPosition();
        rightClawControl = robot._rightClaw.getPosition();

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

            double crrLoopTime = runtime.nanoseconds() / 1000000; // covert to millis
            runtime.reset();

            turretControl = robot._turret.getPosition();
            baseControl = robot._base.getPosition();
            elbowControl = robot._elbow.getPosition();
            wristControl = robot._wrist.getPosition();
            leftClawControl = robot._leftClaw.getPosition();
            rightClawControl = robot._rightClaw.getPosition();

            testArm.setServos(turretControl, baseControl, elbowControl);

            telemetry.addData("loopTime", "{%.3fms}",
                    crrLoopTime);

            telemetry.addData("servos POS->", "{%.0fp %.0fp %.0fp}",
                    turretControl * 100,
                    baseControl * 100,
                    elbowControl * 100);
            telemetry.addData("servos POS->", "{%.0fp %.0fp %.0fp}",
                    wristControl * 100,
                    leftClawControl * 100,
                    rightClawControl * 100);

            telemetry.addData("servos DEG->", "{%.0fdeg %.0fdeg %.0fdeg}",
                    testArm.turretAngle.getPI() / Math.PI * 180,
                    testArm.baseAngle.getPI() / Math.PI * 180,
                    testArm.elbowAngle.getPI() / Math.PI * 180);

            telemetry.addData("servos DEG->", "{%.0fdeg} {%.0fdeg %.0fdeg}",
                    testArm.wristAngle.getPI() / Math.PI * 180,
                    testArm.leftClawAngle.getPI() / Math.PI * 180,
                    testArm.rightClawAngle.getPI() / Math.PI * 180);

            telemetry.addData("coord XYZ->", "{%.0fmm  %.0fmm  %.0fmm}",
                    theArm.getX(),
                    theArm.getY(),
                    theArm.getZ());

            telemetry.addData("claw->", "{%.0fmm}",
                    theArm.getClawMM());

            telemetry.addData("CONTROL lr->", "{%.0fp %.0fp}",
                    lControl * 100,
                    rControl * 100);

            String controlMode = new String("servos");
            if (useDriveControl) controlMode = "drive";
            else if (useAxisControl) controlMode = "axis";

            telemetry.addData("control mode->", "%s",
                    controlMode);

            telemetry.addData("-------->", "");

            if (extendedLogging) {

                telemetry.addData("ARM rtpa2->", "{%.0fmm  %.0fg  %.0fg, %.0fg}",
                        theArm.getR(),
                        theArm.getTeta() / Math.PI * 180,
                        theArm.getPhi() / Math.PI * 180,
                        theArm.getA2() / Math.PI * 180);

                telemetry.addData("ARM servos->", "{%.0fp  %.0fp  %.0fp, %.0fp}",
                        theArm.getTurretServo() * 100,
                        theArm.getBaseServo() * 100,
                        theArm.getElbowServo() * 100,
                        theArm.getWristServo() * 100);

                telemetry.addData("ARM servos DEG->", "{%.0fg  %.0fg  %.0fg, %.0fg}",
                        theArm.turretAngle.getPI() / Math.PI * 180,
                        theArm.baseAngle.getPI() / Math.PI * 180,
                        theArm.elbowAngle.getPI() / Math.PI * 180,
                        theArm.wristAngle.getPI() / Math.PI * 180);

                telemetry.addData("TEST servos DEG->", "{%.0fp  %.0fp  %.0fp, %.0fp}",
                        testArm.getTurretServo() * 100,
                        testArm.getBaseServo() * 100,
                        testArm.getElbowServo() * 100,
                        testArm.getWristServo() * 100);


                telemetry.addData("ARM servos DEG->", "{%.0fg  %.0fg  %.0fg, %.0fg}",
                        testArm.turretAngle.getPI() / Math.PI * 180,
                        testArm.baseAngle.getPI() / Math.PI * 180,
                        testArm.elbowAngle.getPI() / Math.PI * 180,
                        testArm.wristAngle.getPI() / Math.PI * 180);

                telemetry.addData("TEST xyz ->", "{%.0fmm  %.0fmm  %.0fmm}",
                        testArm.getX(),
                        testArm.getY(),
                        testArm.getZ());

                telemetry.addData("TEST rtpa2->", "{%.0fmm  %.0fg  %.0fg, %.0fg}",
                        testArm.getR(),
                        testArm.getTeta() / Math.PI * 180,
                        testArm.getPhi() / Math.PI * 180,
                        testArm.getA2() / Math.PI * 180);
            }

            telemetry.update();

            /**************************** START  OF USE SERVOS CONTROL*****************************/

            if (!useAxisControl && !useDriveControl) {

                // control: BASE
                if (gamepad1.left_stick_y != 0) {
                    baseControl += gamepad1.left_stick_y * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }
                // control: WRIST
                if (gamepad1.left_stick_x != 0) {
                    wristControl += gamepad1.left_stick_x * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }
                // control: ELBOW
                if (gamepad1.right_stick_y != 0) {
                    elbowControl += gamepad1.right_stick_y * servoDefaultSpeed * crrLoopTime;
                    setServos();

                }
                // control: TURRET
                if (gamepad1.right_stick_x != 0) {
                    turretControl += gamepad1.right_stick_x * servoDefaultSpeed * crrLoopTime;
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
                    xyzSetServos();
                    theArm.setServos(turretControl, baseControl, elbowControl);
                    xControl = theArm.getX();
                    yControl = theArm.getY();
                    zControl = theArm.getZ();
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

            if (useAxisControl && !useDriveControl) {

                // control:
                if (gamepad1.left_stick_x != 0) {
                    xControl += gamepad1.left_stick_x * axisDefaultSpeed * crrLoopTime;// * axisDefaultSpeed * crrLoopTime;
                    xyzSetServos();
                }
                // control:
                if (gamepad1.right_stick_y != 0) { //
                    yControl += -gamepad1.right_stick_y * axisDefaultSpeed * crrLoopTime;// * axisDefaultSpeed * crrLoopTime;
                    xyzSetServos();
                }
                // control:
                if (gamepad1.left_stick_y != 0) { //
                    zControl += -gamepad1.left_stick_y * axisDefaultSpeed * crrLoopTime;// * axisDefaultSpeed * crrLoopTime;
                    xyzSetServos();
                }
                // control: claw
                if (gamepad1.right_stick_x != 0) {
                    rightClawControl += gamepad1.right_stick_x * axisDefaultSpeed * crrLoopTime;
                    leftClawControl -= gamepad1.right_stick_x * axisDefaultSpeed * crrLoopTime;
                    xyzSetServos();
                }
                if (gamepad1.back) {
                    theArm.setZeroXYZ();

                    robot._turret.setPosition(theArm.getTurretServo());
                    robot._base.setPosition(theArm.getBaseServo());
                    robot._elbow.setPosition(theArm.getElbowServo());
                    robot._wrist.setPosition(theArm.getWristServo());
                    robot._leftClaw.setPosition(theArm.getLeftClawServo());
                    robot._rightClaw.setPosition(theArm.getRightClawServo());

                    xControl = theArm.getX();
                    yControl = theArm.getY();
                    zControl = theArm.getZ();
                }
                // control:
                if (gamepad1.a) {
                    useAxisControl = false;
                    setServos();
                    theArm.setXYZ(xControl, yControl, zControl);
                    turretControl = theArm.getTurretServo();
                    baseControl = theArm.getBaseServo();
                    elbowControl = theArm.getElbowServo();
                    wristControl = theArm.getWristServo();
                    leftClawControl = theArm.getLeftClawServo();
                    rightClawControl = theArm.getRightClawServo();
                }
                // control:
                if (gamepad1.x) {
                    useDriveControl = true;
                }
            }

            /**************************** START  OF USE DRIVE CONTROL*****************************/

            if (useDriveControl) {

                // control: TURRET
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
                    lControl = 0;// * axisDefaultSpeed * crrLoopTime;
                    rControl = 0;// * axisDefaultSpeed * crrLoopTime;
                    setDrives();
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

    public void setServos() {

        leftClawControl = Range.clip(leftClawControl, theArm.leftClawAngle.minServo, theArm.leftClawAngle.maxServo);
        rightClawControl = Range.clip(rightClawControl, theArm.rightClawAngle.minServo, theArm.rightClawAngle.maxServo);

        robot._leftClaw.setPosition(leftClawControl);
        robot._rightClaw.setPosition(rightClawControl);

        elbowControl = Range.clip(elbowControl, theArm.elbowAngle.minServo, theArm.elbowAngle.maxServo);
        baseControl = Range.clip(baseControl, theArm.baseAngle.minServo, theArm.baseAngle.maxServo);
        wristControl = Range.clip(wristControl, theArm.wristAngle.minServo, theArm.wristAngle.maxServo);
        turretControl = Range.clip(turretControl, theArm.turretAngle.minServo, theArm.turretAngle.maxServo);

        testArm.setServos(turretControl, baseControl, elbowControl);

        /*
        //do a collision check TODO
         if( testArm.collisionCheck(false) ){
            return;
        } */

        // slow down if needed
        double maxServoStep = 1; // 0.1 per servo and step
        double stepCount = 0;
        stepCount = Math.max(stepCount, Math.abs(elbowControl - elbowControlLast) / maxServoStep);
        stepCount = Math.max(stepCount, Math.abs(baseControl - baseControlLast) / maxServoStep);
        stepCount = Math.max(stepCount, Math.abs(wristControl - wristControlLast) / maxServoStep);
        stepCount = Math.max(stepCount, Math.abs(turretControl - turretControlLast) / maxServoStep);

        if (stepCount > 0) {
            for (int i = 0; i < stepCount; i++) {
                double elbowControlStep = elbowControlLast + i * (elbowControl - elbowControlLast) / maxServoStep;
                double baseControlStep = baseControlLast + i * (baseControl - baseControlLast) / maxServoStep;
                double wristControlStep = wristControlLast + i * (wristControl - wristControlLast) / maxServoStep;
                double turretControlStep = turretControlLast + i * (turretControl - turretControlLast) / maxServoStep;
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

    public void xyzSetServos() {

        xControl = Range.clip(xControl, theArm.xMin, theArm.xMax);
        yControl = Range.clip(yControl, theArm.yMin, theArm.yMax);
        zControl = Range.clip(zControl, theArm.zMin, theArm.zMax);

        theArm.setXYZ(xControl, yControl, zControl);

        /*
        //TODO
        if( theArm.collisionCheck( false )){
            return;
        }
         */

        //slow down the move here if needed, also use for collision correction
        double maxStep = 666; // 6mm per axis and step
        double stepCount = 0;
        stepCount = Math.max(stepCount, Math.abs(xControl - xControlLast) / maxStep);
        stepCount = Math.max(stepCount, Math.abs(yControl - yControlLast) / maxStep);
        stepCount = Math.max(stepCount, Math.abs(zControl - zControlLast) / maxStep);

        if (stepCount > 0) {
            Arm stepArm = new Arm();
            stepArm.init();

            for (int i = 0; i < stepCount; i++) {
                stepArm.setXYZ(xControlLast + ((xControl - xControlLast) / stepCount) * i,
                        yControlLast + ((yControl - yControlLast) / stepCount) * i,
                        zControlLast + ((zControl - zControlLast) / stepCount) * i);

                // use the order for minimum colision
                robot._elbow.setPosition(stepArm.getElbowServo());
                robot._base.setPosition(stepArm.getBaseServo());
                robot._turret.setPosition(stepArm.getTurretServo());
                robot._wrist.setPosition(stepArm.getWristServo());
                robot._leftClaw.setPosition(stepArm.getLeftClawServo());
                robot._rightClaw.setPosition(stepArm.getRightClawServo());
                //TODO : add delay
            }
        }

        robot._elbow.setPosition(theArm.getElbowServo());
        robot._base.setPosition(theArm.getBaseServo());
        robot._turret.setPosition(theArm.getTurretServo());
        robot._wrist.setPosition(theArm.getWristServo());
        robot._leftClaw.setPosition(theArm.getLeftClawServo());
        robot._rightClaw.setPosition(theArm.getRightClawServo());

        xControlLast = xControl;
        yControlLast = yControl;
        zControlLast = zControl;
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

