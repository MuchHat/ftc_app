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

@TeleOp(name="OpMode V3", group="Gigi")
// @Disabled
public class Gigi_OpMode_V3 extends LinearOpMode {

    public Gigi_Hardware_V2 robot = new Gigi_Hardware_V2();
    public ArmController armController = new ArmController();
    public ElapsedTime runtime = new ElapsedTime();

    public double turretControl = 0;
    public double baseControl = 0;
    public double elbowControl = 0;
    public double wristControl = 0;
    public double clawControlL = 0;
    public double clawControlR = 0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        armController.init();

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

        armController.setDestinationToCurrent(
                turretControl,
                baseControl,
                elbowControl );

        waitForStart();

        while (opModeIsActive()) {

            double crrLoopTime = runtime.milliseconds();
            runtime.reset();

            turretControl = robot._turret.getPosition();
            baseControl = robot._base.getPosition();
            elbowControl = robot._elbow.getPosition();
            wristControl = robot._wrist.getPosition();
            clawControlL = robot._leftClaw.getPosition();
            clawControlR = robot._rightClaw.getPosition();

            armController.startLoop( turretControl,baseControl, elbowControl );

            String atDestinationStr = new String("moving->");
            if (armController.atDestination) {
                atDestinationStr = "stopped";
                }

            telemetry.addData("loopTime", "{%.0fms}",
                    crrLoopTime );
            telemetry.addData("destination-> distance", "{%.0fmm}",
                    armController.distanceToDestination );
            telemetry.addData("next-> distance", "{%.0fmm}",
                    armController.distanceToNext );
            telemetry.addData("speed", "{%.0fmm}",
                    armController.prevSpeed_mms );

            telemetry.addData("current-> XYZ", "{%.0fmm  %.0fmm  %.0fmm}",
                    armController.current.getX(),
                    armController.current.getY(),
                    armController.current.getZ());
            telemetry.addData("test-> XYZ", "{%.0fmm  %.0fmm  %.0fmm}",
                    armController.test.getX(),
                    armController.test.getY(),
                    armController.test.getZ());
            telemetry.addData("next-> XYZ", "{%.0fmm  %.0fmm  %.0fmm}",
                    armController.next.getX(),
                    armController.next.getY(),
                    armController.next.getZ());
            telemetry.addData("destination-> XYZ", "{%.0fmm  %.0fmm  %.0fmm}",
                    armController.destination.getX(),
                    armController.destination.getY(),
                    armController.destination.getZ());

            telemetry.addData("servos->", "{%.3f  %.3f  %.3f}",
                    turretControl,
                    baseControl,
                    elbowControl);
            telemetry.addData("current-> SRV", "{%.3f  %.3f  %.3f}",
                    armController.current.getTurretServo(),
                    armController.current.getBaseServo(),
                    armController.current.getElbowServo() );
            telemetry.addData("next-> SRV", "{%.3f  %.3f  %.3f}",
                    armController.next.getTurretServo(),
                    armController.next.getBaseServo(),
                    armController.next.getElbowServo() );
            telemetry.addData("destination-> SRV", "{%.3f  %.3f  %.3f}",
                    armController.destination.getTurretServo(),
                    armController.destination.getBaseServo(),
                    armController.destination.getElbowServo() );

            telemetry.addData("current-> RTP", "{%.0fmm  %.0fg  %.0fg  %.0fg}",
                    armController.current.getR(),
                    armController.current.getTeta() * 180 / Math.PI,
                    armController.current.getPhi() * 180 / Math.PI,
                    armController.current.getA2() * 180 / Math.PI);
            telemetry.addData("test-> RTP", "{%.0fmm  %.0fg  %.0fg  %.0fg}",
                    armController.test.getR(),
                    armController.test.getTeta() * 180 / Math.PI,
                    armController.test.getPhi() * 180 / Math.PI,
                    armController.test.getA2() * 180 / Math.PI);

            telemetry.addData("robot is", "%s", atDestinationStr);
            telemetry.update();

            // control: TURRET
            if( gamepad1.left_stick_x != 0 )
            {
                turretControl += gamepad1.left_stick_x * 0.0003 * crrLoopTime;
                turretControl = Range.clip(turretControl, 0.05, 0.95);
                robot._turret.setPosition(turretControl);

            }
            // control: BASE
            if( gamepad1.right_stick_y != 0 )
            {
                baseControl += gamepad1.right_stick_y * 0.0003 * crrLoopTime;
                baseControl = Range.clip(baseControl, 0.05, 0.95);
                robot._base.setPosition(baseControl);

            }
            // control: ELBOW
            if( gamepad1.left_stick_y != 0 )
            {
                elbowControl += gamepad1.left_stick_y * 0.0003 * crrLoopTime;
                elbowControl = Range.clip(elbowControl, 0.05, 0.95);
                robot._elbow.setPosition(elbowControl);

            }
            // control: WRIST
            if( gamepad1.right_stick_x != 0 )
            {
                wristControl += gamepad1.right_stick_x * 0.0003 * crrLoopTime;
                wristControl = Range.clip(wristControl, 0.05, 0.95);
                robot._wrist.setPosition(wristControl);

            }
            // control: CLAW OPEN
            if( gamepad1.right_trigger != 0 )
            {
                clawControlR += gamepad1.right_trigger * 0.0003 * crrLoopTime;
                clawControlL -= gamepad1.right_trigger * 0.0003 * crrLoopTime;
                clawControlR = Range.clip(clawControlR, 0.05, 0.95);
                clawControlL = Range.clip(clawControlL, 0.05, 0.95);
                robot._rightClaw.setPosition(clawControlR);
                robot._leftClaw.setPosition(clawControlL);

            }
            // control: CLAW CLOSE
            if( gamepad1.right_trigger != 0 )
            {
                clawControlR -= gamepad1.right_trigger * 0.0003 * crrLoopTime;
                clawControlL += gamepad1.right_trigger * 0.0003 * crrLoopTime;
                clawControlR = Range.clip(clawControlR, 0.05, 0.95);
                clawControlL = Range.clip(clawControlL, 0.05, 0.95);
                robot._rightClaw.setPosition(clawControlR);
                robot._leftClaw.setPosition(clawControlL);

            }
            // control: HOME
            {
                if (gamepad1.back) {
                    armController.moveToPositionHome();
                }
            }
            // control: LEFT RIGH MM
            {
                if (gamepad1.dpad_up) {
                    armController.moveIncremental( 0, 15, 0 );
                    setServosPerArmController( crrLoopTime );
                }
            }
            // control: LEFT RIGH MM
            {
                if (gamepad1.dpad_down) {
                    armController.moveIncremental( 0, -15, 0 );
                    setServosPerArmController( crrLoopTime );
                }
            }
            // control: LEFT RIGH MM
            {
                if (gamepad1.dpad_right) {
                    armController.moveIncremental( 15, 0, 0 );
                    setServosPerArmController( crrLoopTime );
                }
            }
            // control: LEFT RIGH MM
            {
                if (gamepad1.dpad_left) {
                    armController.moveIncremental( -15, 0, 0 );
                    setServosPerArmController( crrLoopTime );
                }
            }
            // control: LEFT RIGH MM
            {
                if (gamepad1.right_bumper) {
                     armController.moveIncremental( 0, 0, 15 );
                    setServosPerArmController( crrLoopTime );
                }
            }
            // control: LEFT RIGH MM
            {
                if (gamepad1.left_bumper) {
                    armController.moveIncremental( 0, 0, -15 );
                    setServosPerArmController( crrLoopTime );
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



        }
    }

    public void setServosPerArmController( double loopTime ){
        // set the servos per the ARM controller
      //  armController.endLoop( loopTime );

        //robot._turret.setPosition( armController.destination.getTurretServo() );
        //robot._base.setPosition( armController.destination.getBaseServo() );
       // robot._elbow.setPosition( armController.destination.getElbowServo() );
        //robot._wrist.setPosition( armController.destination.getWristVerticalServo() );

        //robot._leftClaw.setPosition( armController.next.getClawServo() );
        //robot._rightClaw.setPosition( 1 - armController.next.getClawServo() );
    }
}

