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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class Gigi_HardwareRobot extends HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;

    public Servo    turret      = null;
    public Servo    bottom      = null;
    public Servo    top         = null;
    public Servo    wrist       = null;
    public Servo    claw_right  = null;
    public Servo    claw_left   = null;

    public final static double turretMin = 0.2;
    public final static double bottomMin = 0.2;
    public final static double topMin = 0.2;
    public final static double wristMin = 0.2;

    public final static double turretMax = 0.8;
    public final static double bottomMax = 0.8;
    public final static double topMax = 0.8;
    public final static double wristMax = 0.8;

    public final static double turretHome = 0.5;
    public final static double bottomHome = 0.5;
    public final static double topHome = 0.5;
    public final static double wristHome = 0.5;

    public final static double turretFront = 0.6;
    public final static double bottomFront = 0.6;
    public final static double topFront = 0.6;
    public final static double wristFront = 0.6;

    public final static double turretFront_plus_x = 0.7;
    public final static double bottomFront_plus_x = 0.6;
    public final static double topFront_plus_x = 0.6;
    public final static double wristFront_plus_x = 0.4;

    public final static double turretFront_minus_x = 0.4;
    public final static double bottomFront_minus_x = 0.6;
    public final static double topFront_minus_x = 0.6;
    public final static double wristFront_minus_x = 0.4;

    public final static double turretFront_plus_z = 0.5;
    public final static double bottomFront_plus_z = 0.7;
    public final static double topFront_plus_z = 0.7;
    public final static double wristFront_plus_z = 0.6;

    public final static double clawOpen = 0.7;
    public final static double clawClose = 0.3;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Gigi_HardwareRobot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init( HardwareMap ahwMap ) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get( DcMotor.class, "left_drive" );
        rightDrive = hwMap.get( DcMotor.class, "right_drive" );
        leftDrive.setDirection( DcMotor.Direction.FORWARD );
        leftDrive.setDirection( DcMotor.Direction.FORWARD );

        // Set all motors to zero power
        leftDrive.setPower( 0 );
        rightDrive.setPower( 0 );

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        rightDrive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        turret      = hwMap.get( Servo.class, "turret" );
        bottom      = hwMap.get( Servo.class, "bottom" );
        top         = hwMap.get( Servo.class, "top" );
        wrist       = hwMap.get( Servo.class, "wrist" );
        claw_right  = hwMap.get( Servo.class, "claw_right" );
        claw_left   = hwMap.get( Servo.class, "claw_left" );

        armHome();
        clawOpen();
    }

    public void armHome() {
        moveArm( turretHome,
                bottomHome,
                topHome,
                wristHome );
    }
    public void armFront() {
        moveArm( turretFront,
                bottomFront,
                topFront,
                wristFront );
    }

    public void armFront_plus_x() {
        moveArm( turretFront_plus_x,
                bottomFront_plus_x,
                topFront_plus_x,
                wristFront_plus_x );
    }

    public void armFront_minus_x() {
        moveArm( turretFront_minus_x,
                bottomFront_minus_x,
                topFront_minus_x,
                wristFront_minus_x );
    }

    public void armFront_plus_z() {
        moveArm( turretFront_plus_z,
                bottomFront_plus_z,
                topFront_plus_z,
                wristFront_plus_z );
    }

    public void clawOpen() {
        claw_right.setPosition( clawOpen );
        claw_left.setPosition( clawOpen );
    }

    public void clawClose() {
        claw_right.setPosition( clawClose );
        claw_left.setPosition( clawClose );
    }

    public void moveArm( double turretNew, double bottomNew, double topNew, double wristNew )
    {
        double turretCrr = turret.getPosition();
        double bottomCrr = bottom.getPosition();
        double topCrr = top.getPosition();
        double wristCrr = wrist.getPosition();

        double maxChange = 0.0;
        maxChange = Math.max( Math.abs( turretNew - turretCrr ), maxChange );
        maxChange = Math.max( Math.abs( bottomNew - bottomCrr ), maxChange );
        maxChange = Math.max( Math.abs( topNew - topCrr ), maxChange );
        maxChange = Math.max( Math.abs( wristNew - wristCrr ), maxChange );

        int steps = (int)( maxChange * 100 );
        if( steps > 66 ) steps = 66;
        if( steps < 3 ) steps = 3;
        int accel = (int)( (double)steps * 0.2 ); // accelate/decelerate first 20%
        if( accel < 2 ) accel = 2;
        if( accel > steps / 2  ) accel = steps / 2;

        for( int i = 0; i < steps; i++ )
        {
            double turretStep = turretCrr + ( ( turretNew - turretCrr) * ( i + 1 ) ) / steps;
            double bottomStep = turretCrr + ( ( bottomNew - bottomCrr) * ( i + 1 ) ) / steps;
            double topStep = turretCrr + ( ( topNew - topCrr ) * ( i + 1 ) ) / steps;
            double wristStep = turretCrr + ( ( wristNew - wristCrr ) * ( i + 1 ) ) / steps;

            turret.setPosition( turretStep );
            bottom.setPosition( bottomStep );
            top.setPosition( topStep );
            wrist.setPosition( wristStep );

            try {
                Thread.sleep( 33, 0 );
            }
            catch( InterruptedException ex ) {
                Thread.currentThread().interrupt();
            }

            long extraWait = 0;
            if( i < accel ) {
                extraWait = ( accel - i ) * 33;
            }
            if( i > (steps - accel ) ){
                extraWait = ( steps - i ) * 33;
            }
            if( extraWait > 0 ){
                try {
                    Thread.sleep( extraWait, 0 );
                }
                catch( InterruptedException ex ) {
                    Thread.currentThread().interrupt();
                }
            }

        }


    }
}
