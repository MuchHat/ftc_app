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

@TeleOp(name="OpMode V5 Working", group="Gigi")
// @Disabled
public class Gigi_OpMode_V5_Working extends LinearOpMode {

    public Gigi_Hardware_V2 robot = new Gigi_Hardware_V2();
    public Arm theArm = null;
    public ElapsedTime runtime = new ElapsedTime();

    public double turretControl = 0;
    public double baseControl = 0;
    public double elbowControl = 0;
    public double wristControl = 0;
    public double clawControlL = 0;
    public double clawControlR = 0;

    public double xControl = 0;
    public double yControl = 0;
    public double zControl = 0;

    boolean useAxisControl = false;

    @Override
    public void runOpMode() {

        robot.init( hardwareMap );
        theArm = new Arm();
        theArm.init();

        robot._turret.setPosition( 0.50 );
        robot._base.setPosition( 0.66 );
        robot._elbow.setPosition( 0.33 );
        robot._wrist.setPosition( 0.50 );
        robot._leftClaw.setPosition( 0.50 );
        robot._rightClaw.setPosition( 0.50 );

        xControl = theArm.getZeroX();
        yControl = theArm.getZeroY();
        zControl = theArm.getZeroZ();

        double servoDefaultSpeed = 0.33 / 1000000; // 0.3 per sec

        turretControl = robot._turret.getPosition();
        baseControl   = robot._base.getPosition();
        elbowControl  = robot._elbow.getPosition();
        wristControl  = robot._wrist.getPosition();
        clawControlL  = robot._leftClaw.getPosition();
        clawControlR  = robot._rightClaw.getPosition();

        waitForStart();

        while ( opModeIsActive() ) {

            double crrLoopTime = runtime.nanoseconds() / 1000;
            runtime.reset();

            turretControl = robot._turret.getPosition();
            baseControl = robot._base.getPosition();
            elbowControl = robot._elbow.getPosition();
            wristControl = robot._wrist.getPosition();
            clawControlL = robot._leftClaw.getPosition();
            clawControlR = robot._rightClaw.getPosition();

            telemetry.addData("loopTime", "{%.3fms}",
                    crrLoopTime / 1000 );

            telemetry.addData("servos control->", "{%.3f  %.3f  %.3f}",
                    turretControl,
                    baseControl,
                    elbowControl );

            telemetry.addData("servos control->", "{%.3f} {%.3f %.3f}",
                    wristControl,
                    clawControlR,
                    clawControlL);

            telemetry.addData("xyz control->", "{%.0fmm  %.0fmm  %.0fmm}",
                    xControl,
                    yControl,
                    zControl );

            telemetry.addData("xyz ->", "{%.0fmm  %.0fmm  %.0fmm}",
                    theArm.getX(),
                    theArm.getY(),
                    theArm.getZ() );

            telemetry.addData("rtp->", "{%.0fmm  %.0fg  %.0fg, %.0fg}",
                    theArm.getR(),
                    theArm.getTeta() / Math.PI * 180,
                    theArm.getPhi() / Math.PI * 180,
                    theArm.getA2() / Math.PI * 180 );

            telemetry.addData("tbew->", "{%.3f  %.3f  %.3f, %.3f}",
                    theArm.getTurretServo(),
                    theArm.getBaseServo(),
                    theArm.getElbowServo(),
                    theArm.getWristVerticalServo() );

            telemetry.update();

            /**************************** START  OF USE AXIS CONTROL*****************************/
            if( !useAxisControl ) {

                theArm.setServos( turretControl, baseControl, elbowControl );

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
                // control: HOME
                if (gamepad1.back) {
                    robot._turret.setPosition(0.50);
                    robot._base.setPosition(0.66);
                    robot._elbow.setPosition(0.33);
                    robot._wrist.setPosition(0.50);
                    robot._leftClaw.setPosition(0.50);
                    robot._rightClaw.setPosition(0.50);
                }
                // control: LEFT RIGH MM
                if (gamepad1.dpad_up) {
                }
                // control: LEFT RIGH MM
                if (gamepad1.dpad_down) {
                }
                // control: LEFT RIGH MM
                if (gamepad1.dpad_right) {
                }
                // control: LEFT RIGH MM
                if (gamepad1.dpad_left) {
                }
                // control: LEFT RIGH MM
                if (gamepad1.right_bumper) {
                }
                // control: LEFT RIGH MM
                if (gamepad1.left_bumper) {
                }
                if (gamepad1.b) {
                    useAxisControl = true;
                }
                // control: FRONT
                if (gamepad1.a) {
                }
                // control: ZERO
                if (gamepad1.x) {
                }
                // control: HOME
                if (gamepad1.y) {
                }
            }
            /**************************** END OF USE DIRECT CONTROL*****************************/

            /**************************** START  OF USE AXIS CONTROL*****************************/
            if( useAxisControl ) {

                theArm.setXYZ( xControl, yControl, zControl );

                // control: TURRET
                if (gamepad1.left_stick_y != 0) {
                    zControl += gamepad1.left_stick_y * servoDefaultSpeed * crrLoopTime * 100;// * axisDefaultSpeed * crrLoopTime;
                    xyzSetServos();
                }
                // control: BASE
                if (gamepad1.right_stick_y != 0) {
                    yControl += -gamepad1.right_stick_y * servoDefaultSpeed * crrLoopTime * 100;// * axisDefaultSpeed * crrLoopTime;
                    xyzSetServos();
                }
                // control: WRIST
                if (gamepad1.right_stick_x != 0) {
                    xControl += -gamepad1.right_stick_x * servoDefaultSpeed * crrLoopTime * 100;// * axisDefaultSpeed * crrLoopTime;
                    xyzSetServos();
                }
                // control: ELBOW
                if (gamepad1.left_stick_x != 0) {
                    clawControlR += gamepad1.left_stick_x * servoDefaultSpeed * crrLoopTime;
                    clawControlL -= gamepad1.left_stick_x * servoDefaultSpeed * crrLoopTime;
                    xyzSetServos();

                }
                if (gamepad1.a) {
                    useAxisControl = false;
                }
             }
            /**************************** END OF USE AXIS CONTROL*****************************/
        }
    }

    public void angleSetServos(){

        clawControlL = Range.clip( clawControlL, theArm.clawLeftAngle.minServo, theArm.clawLeftAngle.maxServo );
        clawControlR = Range.clip( clawControlR, theArm.clawLeftAngle.minServo, theArm.clawLeftAngle.maxServo );

        robot._leftClaw.setPosition( clawControlL );
        robot._rightClaw.setPosition( clawControlR );

        turretControl = Range.clip( turretControl, theArm.turretAngle.minServo, theArm.turretAngle.maxServo);
        robot._turret.setPosition( turretControl );

        elbowControl = Range.clip( elbowControl, theArm.elbowAngle.minServo, theArm.elbowAngle.maxServo );
        robot._elbow.setPosition( elbowControl );

        wristControl = Range.clip( wristControl, theArm.wristVerticalAngle.minServo, theArm.wristVerticalAngle.maxServo );
        robot._wrist.setPosition( wristControl );

        baseControl = Range.clip( baseControl, theArm.baseAngle.minServo, theArm.baseAngle.maxServo );
        robot._base.setPosition( baseControl );
    }

    public void xyzSetServos(){

        theArm.setXYZ( xControl, yControl, zControl );

        double newTurret = theArm.getTurretServo();
        double newBase = theArm.getBaseServo();
        double newElbow = theArm.getElbowServo();
        double newWrist = theArm.getWristVerticalServo();

        //robot._turret.setPosition( turretAngle.getServo( ewTurret ) );
        //robot._base.setPosition( baseAngle.getServo( newBase ) );
        //robot._elbow.setPosition( elbowAngle.getServo( newElbow ) );
        //robot._wrist.setPosition( wristAngle.getServo( newWrist ) );
    }
}

