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
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;

/**
 * This file illustrates the concept of driving a path based on time.
 */

@TeleOp(name="OpMode V3", group="Gigi Linear")
// @Disabled
public class Gigi_OpMode_V3 extends LinearOpMode{

    public  Gigi_Hardware_V2   robot         = new Gigi_Hardware_V2();
    public  ArmController      armController = new ArmController();
    public  ElapsedTime        runtime       = new ElapsedTime();

    public  double             turretControl = 0;
    public  double             baseControl   = 0;
    public  double             elbowControl  = 0;
    public  double             wristControl  = 0;
    public  double             clawControlL  = 0;
    public  double             clawControlR  = 0;

    @Override
    public void runOpMode() {

        robot.init( hardwareMap );
        armController.init();

        robot.leftDrive.setPower( 0 );
        robot.rightDrive.setPower( 0 );
        robot._turret.setPosition( 166 / 255 );
        robot._base.setPosition( 30 / 255 );
        robot._elbow.setPosition( 80 / 255 );
        robot._wrist.setPosition( 80 / 255 );
        robot._leftClaw.setPosition( 0 / 255 );
        robot._rightClaw.setPosition( 140 / 255 );

        waitForStart();

        while( opModeIsActive() ) {

            double crrLoopTime = runtime.milliseconds();
            runtime.reset();

            turretControl = robot._turret.getPosition();
            baseControl = robot._base.getPosition();
            elbowControl = robot._elbow.getPosition();
            wristControl = robot._wrist.getPosition();
            clawControlL = robot._leftClaw.getPosition();
            clawControlR = robot._rightClaw.getPosition();

            armController.startLoop(turretControl, baseControl, elbowControl);

            //String atDestinationStr = new String( "moving->" );
            //if( armController.atDestination ){
            //    atDestinationStr = "stopped";
            //}

            telemetry.addData("current -> turret", "%.2f", armController.getCurrentTurretServo());
            telemetry.addData("current -> base", "%.2f", armController.getCurrentBaseServo());
            telemetry.addData("current -> elbow", "%.2f", armController.getCurrentElbowServo());
            telemetry.addData("current -> r", "%.2fmm", armController.getCurrentR());
            telemetry.addData("current -> teta", "%.2fpi", armController.getCurrentTeta());
            telemetry.addData("current -> phi", "%.2fpi", armController.getCurrentPhi());
            telemetry.addData("current -> x axis", "%.2fmm", armController.getCurrentX());
            telemetry.addData("current -> y axis", "%.2fmm", armController.getCurrentY());
            telemetry.addData("current -> z axis", "%.2fmm", armController.getCurrentX());
            telemetry.addData("test->turret", "%.2fpi", armController.getCurrentTestTurretServo());
            telemetry.addData("test->base", "%.2fpi", armController.getCurrentTestBaseServo());
            telemetry.addData("test->elbow", "%.2fpi", armController.getCurrentTestELbowServo());
            telemetry.update();

            // control: TURRET
            {
                turretControl += gamepad1.left_stick_x * 0.0003 * crrLoopTime;
                turretControl = Range.clip(turretControl, 0.15, 0.95);
                robot._turret.setPosition(turretControl);
            }
            // control: BASE
            {
                baseControl += gamepad1.right_stick_y * 0.0003 * crrLoopTime;
                baseControl = Range.clip(baseControl, 0.15, 0.95);
                robot._base.setPosition(baseControl);
            }
            // control: ELBOW
            {
                elbowControl += gamepad1.left_stick_y * 0.0003 * crrLoopTime;
                elbowControl = Range.clip(elbowControl, 0.15, 0.95);
                robot._elbow.setPosition(elbowControl);
            }
            // control: WRIST
            {
                wristControl += gamepad1.right_stick_x * 0.0003 * crrLoopTime;
                wristControl = Range.clip(wristControl, 0.15, 0.95);
                robot._wrist.setPosition(wristControl);
            }
            // control: CLAW OPEN
            {
                clawControlR += gamepad1.right_trigger * 0.0003 * crrLoopTime;
                clawControlL -= gamepad1.right_trigger * 0.0003 * crrLoopTime;
                clawControlR = Range.clip(clawControlR, 0.15, 0.95);
                clawControlL = Range.clip(clawControlL, 0.15, 0.95);
                robot._rightClaw.setPosition(clawControlR);
                robot._leftClaw.setPosition(clawControlL);
            }
            // control: CLAW CLOSE
            {
                clawControlR -= gamepad1.right_trigger * 0.0003 * crrLoopTime;
                clawControlL += gamepad1.right_trigger * 0.0003 * crrLoopTime;
                clawControlR = Range.clip(clawControlR, 0.15, 0.95);
                clawControlL = Range.clip(clawControlL, 0.15, 0.95);
                robot._rightClaw.setPosition(clawControlR);
                robot._leftClaw.setPosition(clawControlL);
            }
            // control: HOME
            {
                if (gamepad1.a) {
                    robot._turret.setPosition(166 / 255);
                    robot._base.setPosition(30 / 255);
                    robot._elbow.setPosition(80 / 255);
                    robot._wrist.setPosition(80 / 255);
                    robot._leftClaw.setPosition(0 / 255);
                    robot._rightClaw.setPosition(140 / 255);
                }
            }
            // control: LEFT RIGH MM
            {
                if (gamepad1.dpad_up) {
                    armController.moveIncremental(0, 15, 0);
                }
            }
            // control: LEFT RIGH MM
            {
                if (gamepad1.dpad_down) {
                    armController.moveIncremental(0, -15, 0);
                }
            }
            // control: LEFT RIGH MM
            {
                if (gamepad1.dpad_right) {
                    armController.moveIncremental(15, 0, 0);
                }
            }
            // control: LEFT RIGH MM
            {
                if (gamepad1.dpad_left) {
                    armController.moveIncremental(-15, 0, 0);
                }
            }
            // control: LEFT RIGH MM
            {
                if (gamepad1.right_bumper) {
                    armController.moveIncremental(0, 0, 15);
                }
            }
            // control: LEFT RIGH MM
            {
                if (gamepad1.left_bumper) {
                    armController.moveIncremental(0, 0, -15);
                }
            }
            // control: FRONT
            {
                if (gamepad1.a) {
                    armController.moveToPositionFront();
                }
            }
            // control: ZERO
            {
                if (gamepad1.x) {
                    armController.moveToPositionZero();
                }
            }
            // control: HOME
            {
                if (gamepad1.y) {
                    armController.moveToPositionHome();
                }
            }

            // set the servos per the ARM controller
            armController.endLoop(crrLoopTime);

            /* TODO enable below later
            robot._turret.setPosition( armController.next.turretAngle.angleServo );
            robot._base.setPosition( armController.next.baseAngle.angleServo );
            robot._elbow.setPosition( armController.next.elbowAngle.angleServo );
            robot._wrist.setPosition( armController.next.clawVerticalAngle.angleServo );

            robot._leftClaw.setPosition( armController.next.clawOpeningAngle.angleServo );
            robot._rightClaw.setPosition( 140 - armController.next.clawOpeningAngle.angleServo );
            TODO END */
        }
     }
}
