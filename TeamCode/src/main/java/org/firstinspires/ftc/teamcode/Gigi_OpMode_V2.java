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

        // Send telemetry message to signify robot waiting;
        telemetry.addData("runOpMode", "waitForStart");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.leftDrive.setPower( 0 );
        robot.rightDrive.setPower( 0 );

        boolean useAnimation = false;

        if( useAnimation ) {

        }

        if( !useAnimation ) {
            robot.elbow.setPositionHome();
            robot.base.setPositionHome();
            robot.turret.setPositionHome();
            robot.wrist.setPositionHome();
            robot.leftClaw.setPositionHome();
            robot.rightClaw.setPositionHome();
        }

        while (opModeIsActive() ) {

            if( gamepad1.right_stick_x > 0 ){
                robot.base.moveServoIncremental( gamepad1.right_stick_x * 0.1 );
            }
            if( gamepad1.right_stick_y > 0 ){
                robot.wrist.moveServoIncremental( gamepad1.right_stick_y * 0.1 );
            }
            if( gamepad1.left_stick_x > 0 ){
                robot.elbow.moveServoIncremental( gamepad1.left_stick_x * 0.1 );
            }
            if( gamepad1.left_stick_y > 0 ){
                robot.turret.moveServoIncremental( gamepad1.left_stick_y * 0.1 );
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
            sleep( 111 );
        }

        robot.elbow.setPosition( 0 );
        robot.base.setPosition( 0 );
        robot.turret.setPosition( 0 );
        robot.wrist.setPosition( 0 );
        robot.leftClaw.setPosition( 0 );
        robot.rightClaw.setPosition( 0 );

        sleep( 999999 );
    }
}
