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

@TeleOp(name="OpMode V5", group="Gigi")
// @Disabled
public class Gigi_OpMode_V5 extends LinearOpMode {

    public Gigi_Hardware_V2 robot = new Gigi_Hardware_V2();
    public ArmCalculator theArmCalculator = null;
    public ServoCalculator theServoCalculator = new ServoCalculator();
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

    AngleCalculator turretAngle = null;
    AngleCalculator baseAngle = null;
    AngleCalculator elbowAngle = null;
    AngleCalculator wristAngle = null;
    AngleCalculator clawRightAngle = null;
    AngleCalculator clawLeftAngle  = null;

    boolean useAxisControl = false;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        turretAngle = new AngleCalculator();
        turretAngle.init( 0.140, 0.644, 0.05, 0.897 );

        baseAngle = new AngleCalculator();
        baseAngle.init( 0.25, 0.75, 0.05, 0.95 );

        elbowAngle = new AngleCalculator();
        elbowAngle.init(0.404, 0.950, 0.20, 0.95); //elbow setup

        wristAngle = new AngleCalculator();
        wristAngle.init(1.1775, 0.6105, 0.32, 0.89);

        clawRightAngle = new AngleCalculator();
        clawRightAngle.init(0.221, -0.437, 0.221, 0.55); // right claw setup

        clawLeftAngle = new AngleCalculator();
       // clawLeftAngle.init((0.818, 1.582, 0.436, 0.818); // left claw setup

        theArmCalculator = new ArmCalculator();

        robot._turret.setPosition( 0.50 );
        robot._base.setPosition( 0.66 );
        robot._elbow.setPosition( 0.33 );
        robot._wrist.setPosition( 0.50 );
        robot._leftClaw.setPosition( 0.50 );
        robot._rightClaw.setPosition( 0.50 );

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

            xControl = 0;
            yControl = 0;
            zControl = 0;

            double servoDefaultSpeed = 0.33 / 1000000; // 0.3 per sec

            telemetry.addData("loopTime", "{%.3fms}",
                    crrLoopTime/1000 );

            telemetry.addData("xyz->", "{%.0fmm  %.0fmm  %.0fmm}",
                    xControl,
                    yControl,
                    zControl );
            telemetry.addData("servos->", "{%.3f  %.3f  %.3f}",
                    turretControl,
                    baseControl,
                    elbowControl);
            telemetry.update();

            /**************************** START  OF USE AXIS CONTROL*****************************/
            if( useAxisControl ) {

                // control: BASE
                if (gamepad1.left_stick_y != 0) {
                        baseControl += gamepad1.left_stick_y * servoDefaultSpeed * crrLoopTime;
                        baseControl = Range.clip(baseControl, 0.05, 0.95 );
                        robot._base.setPosition( baseControl);
                }
                // control: WRIST
                if (gamepad1.left_stick_x != 0) {
                    wristControl += gamepad1.left_stick_x * servoDefaultSpeed * crrLoopTime;
                    wristControl = Range.clip(wristControl, 0.05, 0.95 );
                    robot._wrist.setPosition( wristControl);
                }
                // control: ELBOW
                    if (gamepad1.left_stick_y != 0) {
                        elbowControl += gamepad1.left_stick_y * servoDefaultSpeed * crrLoopTime;
                        elbowControl = Range.clip(elbowControl, 0.05, 0.95 );
                        robot._elbow.setPosition( elbowControl);
                }
                // control: TURRET
                if (gamepad1.left_stick_x != 0) {
                    turretControl += gamepad1.left_stick_x * servoDefaultSpeed * crrLoopTime;
                    turretControl = Range.clip(turretControl, 0.05, 0.95 );
                    robot._turret.setPosition( turretControl);
                }
                // control: CLAW CLOSE
                    if (gamepad1.left_trigger != 0) {
                        clawControlL -= gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                        clawControlR += gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                        clawControlL = Range.clip(clawControlL, 0.05, 0.95 );
                        clawControlR = Range.clip(clawControlR, 0.05, 0.95 );
                        robot._leftClaw.setPosition( clawControlL);
                        robot._rightClaw.setPosition( clawControlR);
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
                        useAxisControl = false;
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

                // control: TURRET
                if (gamepad1.left_stick_y != 0) {
                    zControl += gamepad1.left_stick_y * servoDefaultSpeed * crrLoopTime * 100;// * axisDefaultSpeed * crrLoopTime;
                    xyzSetServos();
                }
                // control: BASE
                if (gamepad1.right_stick_y != 0) {
                    yControl += -gamepad1.right_stick_y * servoDefaultSpeed * crrLoopTime * 100;// * axisDefaultSpeed * crrLoopTime;
                    xyzSetServos();                }
                // control: WRIST
                if (gamepad1.right_stick_x != 0) {
                    xControl += -gamepad1.right_stick_x * servoDefaultSpeed * crrLoopTime * 100;// * axisDefaultSpeed * crrLoopTime;
                    xyzSetServos();                }
                // control: ELBOW
                if (gamepad1.left_stick_x != 0) {
                    clawControlR += gamepad1.left_stick_x * servoDefaultSpeed * crrLoopTime;
                    clawControlL -= gamepad1.left_stick_x * servoDefaultSpeed * crrLoopTime;
                    clawControlR = Range.clip(clawControlR, 0.05, 0.95);
                    xyzSetServos();

                }
                // control: CLAW OPEN
                if (gamepad1.right_trigger != 0) {


                }
                // control: CLAW CLOSE
                if (gamepad1.right_trigger != 0) {
                 }
                // control: HOME
                {
                    if (gamepad1.back) {
                    }
                }
                // control: LEFT RIGH MM
                {
                    if (gamepad1.dpad_up) {
                        //armController.moveIncremental( 0, 15 * axisDefaultSpeed * crrLoopTime, 0 );
                        //setServosFromArmController();
                    }
                }
                // control: LEFT RIGH MM
                {
                    if (gamepad1.dpad_down) {
                        //armController.moveIncremental( 0, -15 * axisDefaultSpeed *  crrLoopTime, 0 );
                        //setServosFromArmController();
                    }
                }
                // control: LEFT RIGH MM
                {
                    if (gamepad1.dpad_right) {
                        //armController.moveIncremental( 15 * axisDefaultSpeed *  crrLoopTime, 0, 0 );
                        //setServosFromArmController();
                    }
                }
                // control: LEFT RIGH MM
                {
                    if (gamepad1.dpad_left) {
                        //armController.moveIncremental( -15 * axisDefaultSpeed *  crrLoopTime, 0, 0 );
                        //setServosFromArmController();
                    }
                }
                // control: LEFT RIGH MM
                {
                    if (gamepad1.right_bumper) {
                        //armController.moveIncremental( 0, 0, 15 * axisDefaultSpeed *  crrLoopTime );
                        //setServosFromArmController();
                    }
                }
                // control: LEFT RIGH MM
                {
                    if (gamepad1.left_bumper) {
                        //armController.moveIncremental( 0, 0, -15 * axisDefaultSpeed *  crrLoopTime );
                        //setServosFromArmController();
                    }
                }
                // control: FRONT
                {
                    if (gamepad1.a) {
                        //armController.moveToPositionFront();
                        useAxisControl = false;
                    }
                }                // control: FRONT
                {
                    if (gamepad1.b) {
                        //armController.moveToPositionFront();
                        useAxisControl = true;
                    }
                }
                // control: ZERO
                {
                    if (gamepad1.x) {
                    }
                }
                // control: HOME
                {
                    if (gamepad1.y) {
                    }
                }
            }
            /**************************** END OF USE AXIS CONTROL*****************************/

        }
    }

    public void xyzSetServos(){

        robot._turret.setPosition( turretAngle.getServo( theArmCalculator.getTurret( xControl, yControl, zControl ) ) );
        robot._base.setPosition( baseAngle.getServo( theArmCalculator.getBase( xControl, yControl, zControl ) ) );
        robot._elbow.setPosition( elbowAngle.getServo( theArmCalculator.getElbow( xControl, yControl, zControl ) ) );
        robot._wrist.setPosition( wristAngle.getServo( theArmCalculator.getWrist( xControl, yControl, zControl ) ) );
    }
}

