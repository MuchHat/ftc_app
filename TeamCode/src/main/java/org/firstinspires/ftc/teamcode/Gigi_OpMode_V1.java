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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@TeleOp(name="Gigi_OpMode_Trigonometric", group="Gigi")
@Disabled
public class Gigi_OpMode_V1 extends LinearOpMode {

    /* Declare OpMode members. */
    Gigi_Hardware_V1 robot = new Gigi_Hardware_V1();   // Use a Pushbot's hardware
                                                                  // could also use HardwarePushbotMatrix class.
    double lastGamePadRead = 0;
    double newGamePadSessionStart = 0;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init( hardwareMap );

        // Send telemetry message to signify robot waiting;
        telemetry.addData( "Say", "Hello Driver" );    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData( "Say", "Started" );    //
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while ( opModeIsActive() ) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

            if( !(  ( drive == 0.0 ) &&
                    ( turn == 0.0 ) &&
                    ( robot.leftDrive.getPower() == 0 ) &&
                    ( robot.rightDrive.getPower() == 0 ) ) ){

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
                robot.leftDrive.setPower(left);
                robot.rightDrive.setPower(right);
            }

            if (gamepad1.a) {
                telemetry.addData("RobotMove", "Arm Front");
                telemetry.update();
                robot.armFront();
            }

            if (gamepad1.x) {
                telemetry.addData("RobotMove", "Arm Front Mins X");
                telemetry.update();
                robot.armFront_minus_x();
            }

            if (gamepad1.b) {
                telemetry.addData("RobotMove", "Arm Front Plus X");    //
                telemetry.update();
                robot.armFront_plus_x();
            }

            if (gamepad1.y) {
                telemetry.addData("RobotMove", "Arm Front Plus Z");    //
                telemetry.update();
                robot.armFront_plus_z();
            }

            if (gamepad1.right_bumper) {
                telemetry.addData("RobotMove", "Claw Open");    //
                telemetry.update();
                robot.clawOpen();
            }

            if (gamepad1.left_bumper) {
                telemetry.addData("RobotMove", "Claw Close");    //
                telemetry.update();
                robot.clawClose();
            }

            if (gamepad1.back) {
                robot.armHome();
                telemetry.addData("RobotMove", "Claw Close");    //
                telemetry.update();
            }

            // move arm by position

            double newGamePadRead = System.currentTimeMillis();
            double millisElapsed = newGamePadRead - lastGamePadRead;

            lastGamePadRead = newGamePadRead;
            if( millisElapsed < 0 ){
                millisElapsed = 0;
            }
            boolean newSession = false;

            if( millisElapsed > 555 ){
                // this is new session
                newGamePadSessionStart = System.currentTimeMillis();
                newSession = true;
            }
            double millisInSession = newGamePadRead - newGamePadSessionStart;

            // only reposition 3 times / sec
            if( millisElapsed > 333 || newSession ) {

                // the inputs are proportional with the speed of change
                double speedX = gamepad2.right_stick_x;
                double speedY = -gamepad2.right_stick_y;
                double speedZ = 0;

                if (gamepad2.dpad_up) {
                    speedZ = 11;
                }
                if (gamepad2.dpad_down) {
                    speedZ = -11;
                }

                // reduce the spped in half the first 555 millis for short moves
                if( millisInSession < 555 )
                {
                    speedX /= 2;
                    speedY /= 2;
                    speedZ /= 2;
                }
                // conver 0 to 1 into mm per sec, max on input is 10 cm  / sec
                speedX = speedX * 111;
                speedY = speedY * 111;
                speedZ = speedZ * 111;

                //adjust the changes not to exceed speed
                speedX = Math.min( speedX, 111 );
                speedY = Math.min( speedY, 111 );
                speedZ = Math.min( speedZ, 111 );

                speedX = Math.max( speedX, -111 );
                speedY = Math.max( speedY, -111 );
                speedZ = Math.max( speedZ, -111 );

                robot.computeCurrentCoordinates();

                // time elapsed is in millis
                // speed is in mm/sec
                double changeX = ( millisElapsed * speedX ) / 1000;
                double changeY = ( millisElapsed * speedY ) / 1000;
                double changeZ = ( millisElapsed * speedZ ) / 1000;

                double newX = robot.currentCoordinateX + changeX;
                double newY = robot.currentCoordinateY + changeY;
                double newZ = robot.currentCoordinateZ + changeZ;

                robot.moveArmByCoordinates( newX, newY, newZ );

            }
            telemetry.addData( "RUNNING  at ", "%5.2f", (double)System.currentTimeMillis() / 1000 );
            telemetry.addData( "TURRET   at ", "%5.2f", robot.getTurret() );
            telemetry.addData( "BASE     at ", "%5.2f", robot.getBase());
            telemetry.addData( "ELBOW    at ", "%5.2f", robot.getElbow() );
            telemetry.addData( "WRIST    at ", "%5.2f", robot.getWrist() );
            telemetry.update();
        }

        // Pace this loop so jaw action is reasonable speed.
        sleep( 50 );
    }
}
