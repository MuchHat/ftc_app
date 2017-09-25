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

import android.animation.ValueAnimator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Gigi: OpMode_Template", group="Gigi")
// @Disabled
public class Gigi_OpMode_Template extends LinearOpMode {

    /* Declare OpMode members. */
    Gigi_HardwareRobot_Template robot = new Gigi_HardwareRobot_Template();
    double servoInt = 0.000;
    double homeTurret = 170;
    double homeBottom = 10;
    double homeTop = 50;
    double homeWrist = 70;
    double homeLeftClaw = 20;
    double homeRightClaw = 200;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        boolean firstrun = true;

        if(firstrun)
        {
            robot.turret.setPosition( homeTurret / 255 );
            robot.base.setPosition( homeBottom / 255 );
            robot.elbow.setPosition( homeTop /255 );
            robot.wrist.setPosition( homeWrist /255 );
            robot.leftClaw.setPosition( homeLeftClaw /255 );
            robot.rightClaw.setPosition( homeRightClaw /255 );
            firstrun = false;
        }

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init( hardwareMap );

        // Send telemetry message to signify robot waiting;
        telemetry.addData( "Say", "Hello Driver" );    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            robot.leftDrive.setPower( left );
            robot.rightDrive.setPower( right );

            if (gamepad1.a) {
                telemetry.addData( "GamePad 1", "A" );
                telemetry.update();

                // fill in here the actual home positions


                robot.turret.setPosition( homeTurret / 255 );
                robot.base.setPosition( homeBottom / 255 );
                robot.elbow.setPosition( homeTop /255 );
                robot.wrist.setPosition( homeWrist /255 );
                robot.leftClaw.setPosition( homeLeftClaw /255 );
                robot.rightClaw.setPosition( homeRightClaw /255 );

                telemetry.addData( "turret at ", "%5.2f", robot.turret.getPosition() );
                telemetry.addData( "bottom at ", "%5.2f", robot.base.getPosition() );
                telemetry.addData( "top at ", "%5.2f", robot.elbow.getPosition() );
                telemetry.addData( "wrist at ", "%5.2f", robot.wrist.getPosition() );
                telemetry.addData( "left claw at ", "%5.2f", robot.leftClaw.getPosition() );
                telemetry.addData( "right claw at ", "%5.2f", robot.rightClaw.getPosition() );
                telemetry.update();
            }

            if(gamepad1.right_stick_x != 0){
                // move arm forward

                // fill in here the limit positions
                double minBottom = 0;
                double maxBottom = 255;
                double speedBottom = 0.05;  //speed when stick is at max

                // servo could be backwards moving from a big number to a small one
                double dirBottom = maxBottom > minBottom ? 1.0 : -1.0;

                double crrBottom = robot.base.getPosition();
                double changeBottom = speedBottom * dirBottom * gamepad1.right_stick_x;
                double newBottom = crrBottom + changeBottom;

                if( dirBottom > 0 ){
                    if (newBottom * 255 > maxBottom) newBottom = maxBottom / 255;
                    if (newBottom * 255 < minBottom) newBottom = minBottom / 255;
                }
                if( dirBottom < 0 ){
                    if (newBottom * 255 < maxBottom) newBottom = maxBottom / 255;
                    if (newBottom * 255 > minBottom) newBottom = minBottom / 255;
                }

                robot.base.setPosition( newBottom );

                telemetry.addData( "bottom at ", "%5.2f", robot.base.getPosition() );
                telemetry.update();
            }
            if (gamepad1.x) {
                telemetry.addData( "GamePad 1", "X" );
                telemetry.update();

                moveServo(50,150,255);
            }

            if (gamepad1.b) {
                telemetry.addData( "GamePad 1", "B" );
                telemetry.update();

                // ADD CODE HERE
            }

            if (gamepad1.y) {
                telemetry.addData( "GamePad 1", "Y" );
                telemetry.update();

                // ADD CODE HERE
            }

            if (gamepad1.right_bumper) {
                telemetry.addData( "GamePad 1", "R bumper" );
                telemetry.update();

                // ADD CODE HERE
            }

            if (gamepad1.left_bumper) {
                telemetry.addData( "GamePad 1", "L bumper" );
                telemetry.update();

                // ADD CODE HERE
            }

            if (gamepad1.back) {
                telemetry.addData( "GamePad 1", "Back" );
                telemetry.update();

                // ADD CODE HERE
            }
        }
        sleep( 50 );
    }
    public void moveServo(int initialValue, int finalValue, final int max) {
        ValueAnimator valueAnimator = ValueAnimator.ofInt(initialValue, finalValue);
        valueAnimator.setDuration(750);
        valueAnimator.addUpdateListener(new ValueAnimator.AnimatorUpdateListener() {
            @Override
            public void onAnimationUpdate(ValueAnimator valueAnimator) {
                servoInt = Double.valueOf(valueAnimator.getAnimatedValue().toString());
                servoInt /= max;
                robot.turret.setPosition(servoInt);
            }
        });
        valueAnimator.start();

    }
}
