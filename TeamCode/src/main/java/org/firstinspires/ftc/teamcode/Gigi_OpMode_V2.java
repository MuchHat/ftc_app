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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 */

@Autonomous(name="OpMode V2", group="Gigi")
// @Disabled
public class Gigi_OpMode_V2 extends LinearOpMode {

    /* Declare OpMode members. */
    Gigi_Hardware_V2        robot   = new Gigi_Hardware_V2();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init( hardwareMap );

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.leftDrive.setPower( 0 );
        robot.rightDrive.setPower( 0 );

        // Send telemetry message
        telemetry.addData( "turret at ", "%f1.3", robot.turret.getPosition() );
        telemetry.addData( "base at ", "%f1.3", robot.turret.getPosition() );
        telemetry.addData( "elbow at", "%f1.3", robot.turret.getPosition() );
        telemetry.addData( "wrist at", "%f1.3", robot.turret.getPosition() );
        telemetry.addData( "left claw at", "%f1.3", robot.turret.getPosition() );
        telemetry.addData( "right claw at", "%f1.3", robot.turret.getPosition() );

        // move the arm to home position
        boolean useAnimationLinear = true;
        boolean useAnimationLibrary = false;
        boolean useNoAnimation = false;

        if( useAnimationLinear ) {
            double stepCount = 12;
            double stepWait = 33;
            double rampCount = 4;

            double stepIncrementTurret = robot.turret.getHomePosIncrement() / stepCount;
            double stepIncrementBase = robot.base.getHomePosIncrement() / stepCount;
            double stepIncrementElbow = robot.elbow.getHomePosIncrement() / stepCount;
            double stepIncrementWrist = robot.wrist.getHomePosIncrement() / stepCount;
            double stepIncrementLeftClaw = robot.leftClaw.getHomePosIncrement() / stepCount;
            double stepIncrementRightClaw = robot.rightClaw.getHomePosIncrement() / stepCount;

            for( int step = 0; step < stepCount; step++ ){
                robot.elbow.moveServoIncremental( stepIncrementElbow );
                robot.base.moveServoIncremental( stepIncrementBase );
                robot.turret.moveServoIncremental( stepIncrementTurret );
                robot.wrist.moveServoIncremental( stepIncrementWrist );
                robot.leftClaw.moveServoIncremental( stepIncrementLeftClaw );
                robot.rightClaw.moveServoIncremental( stepIncrementRightClaw );

                double crrWait = stepWait;
                if( step < rampCount ){
                    crrWait += stepWait * ( rampCount - step );
                }
                if( step > stepCount - rampCount ){
                    crrWait += stepWait *( step - ( stepCount - rampCount ) );
                }
                sleep( (long)crrWait );
            }
        }
        else if( useAnimationLibrary )
        {
/*
        double animationTime = (long)( Math.abs( pos - theServo.getPosition() ) * 1666 );

        ValueAnimator valueAnimator = ValueAnimator.ofInt((int)( theServo.getPosition() * 1000 ), (int)( getAdjustedPositionSafe( pos ) * 1000 ) );
        valueAnimator.setDuration( 666 );
        valueAnimator.addUpdateListener(new ValueAnimator.AnimatorUpdateListener() {
            @Override
            public void onAnimationUpdate(ValueAnimator valueAnimator) {
                double crrPos = Double.valueOf(valueAnimator.getAnimatedValue().toString()) / 1000;
                theServo.setPosition( crrPos );
            }
        });
        valueAnimator.start();
        while( valueAnimator.isRunning() ){

        }
        theServo.setPosition( Range.clip( getAdjustedPositionSafe( pos ), 0.0, 1.0 ) );
 */
        }
        else if( useNoAnimation ){
            robot.elbow.setPositionHome();
            robot.base.setPositionHome();
            robot.turret.setPositionHome();
            robot.wrist.setPositionHome();
            robot.leftClaw.setPositionHome();
            robot.rightClaw.setPositionHome();
        }

        while ( opModeIsActive() ) {

            if( gamepad1.right_stick_y != 0 ){
                robot.base.moveServoIncremental( gamepad1.right_stick_y * 0.1 );
                robot.elbow.moveServoIncremental( gamepad1.right_stick_y * 0.1 * 0.3 );
            }
            if( gamepad1.right_stick_x != 0 ){
                robot.wrist.moveServoIncremental( gamepad1.right_stick_x * 0.1 );
            }
            if( gamepad1.left_stick_y > 0 ){
                robot.turret.moveServoIncremental( gamepad1.left_stick_y * 0.1 );
                robot.leftClaw.moveServoIncremental( gamepad1.left_stick_y * 0.1 );
                robot.rightClaw.moveServoIncremental( -gamepad1.left_stick_y * 0.1 );
            }
            if( gamepad1.left_stick_x > 0 ){
                robot.elbow.moveServoIncremental( gamepad1.left_stick_x * 0.1 );
                robot.base.moveServoIncremental( -gamepad1.left_stick_x * 0.1 * 0.3 );
            }
            if( gamepad1.right_trigger > 0) {
                robot.leftClaw.moveServoIncremental( gamepad1.right_trigger  * 0.1 );
                robot.rightClaw.moveServoIncremental( gamepad1.right_trigger  * 0.1 );
            }
            if( gamepad1.left_trigger > 0) {
                robot.leftClaw.moveServoIncremental( -gamepad1.left_trigger  * 0.1 );
                robot.rightClaw.moveServoIncremental( +gamepad1.left_trigger  * 0.1 );
            }
            if( gamepad1.back ) {
                robot.elbow.setPositionHome();
                robot.base.setPositionHome();
                robot.turret.setPositionHome();
                robot.wrist.setPositionHome();
                robot.leftClaw.setPositionHome();
                robot.rightClaw.setPositionHome();
            }
            if( gamepad1.a ) {
                robot.elbow.setPositionHome();
                robot.base.setPositionHome();
                robot.turret.setPositionHome();
                robot.wrist.setPositionHome();
                robot.leftClaw.setPositionHome();
                robot.rightClaw.setPositionHome();
            }
            if( gamepad1.b ) {
                robot.elbow.setPositionHome();
                robot.base.setPositionHome();
                robot.turret.setPositionHome();
                robot.wrist.setPositionHome();
                robot.leftClaw.setPositionHome();
                robot.rightClaw.setPositionHome();
            }
            if( gamepad1.x ) {
                robot.elbow.setPositionHome();
                robot.base.setPositionHome();
                robot.turret.setPositionHome();
                robot.wrist.setPositionHome();
                robot.leftClaw.setPositionHome();
                robot.rightClaw.setPositionHome();
            }
            if( gamepad1.y ) {
                robot.elbow.setPositionHome();
                robot.base.setPositionHome();
                robot.turret.setPositionHome();
                robot.wrist.setPositionHome();
                robot.leftClaw.setPositionHome();
                robot.rightClaw.setPositionHome();
            }
            telemetry.addData( "turret at ", "%f1.3", robot.turret.getPosition() );
            telemetry.addData( "base at ", "%f1.3", robot.turret.getPosition() );
            telemetry.addData( "elbow at", "%f1.3", robot.turret.getPosition() );
            telemetry.addData( "wrist at", "%f1.3", robot.turret.getPosition() );
            telemetry.addData( "left claw at", "%f1.3", robot.turret.getPosition() );
            telemetry.addData( "right claw at", "%f1.3", robot.turret.getPosition() );
            telemetry.update();

            sleep( 111 );
        }

        sleep( 999 );

        // move the arm to rest position
        robot.elbow.setPosition( 0.1 );
        robot.base.setPosition( 0.1 );
        robot.turret.setPosition( 0.1 );
        robot.wrist.setPosition( 0.1 );
        robot.leftClaw.setPosition( 0.1 );
        robot.rightClaw.setPosition( 0.1 );

        // wait 30 mins
        sleep( 1999000 );
    }
}
