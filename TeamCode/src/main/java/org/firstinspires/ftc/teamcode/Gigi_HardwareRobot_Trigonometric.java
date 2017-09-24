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
public class Gigi_HardwareRobot_Trigonometric extends HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;

    public Servo    turret      = null;
    public Servo    bottom      = null;
    public Servo    top         = null;
    public Servo    wrist       = null;
    public Servo    clawRight   = null;
    public Servo    clawLeft    = null;

    public double currentCoordinateX = 0;
    public double currentCoordinateY = 0;
    public double currentCoordinateZ = 0;

    // alpha is number between 0 and 180 indicating the pos of min for the servo
    // min, max, home are numbers between 0 to 255

    // TODO populate servo info
    public final static double turretRef_A0_A180_Min_Max[ ] = { 0, 255, 0, 255 };
    public final static double bottomRef_A0_A180_Min_Max[ ] = { 0, 255, 0, 255 };
    public final static double topRef_A0_A180_Min_Max[ ] = { 0, 255, 0, 255 };
    public final static double wristUpDownRef_A0_A180_Min_Max[ ] = { 0, 255, 0, 255 };
    public final static double wristLeftRightRef_A0_A180_Min_Max[ ] = { 0, 255, 0, 255 };
    public final static double clawLeftRef_A0_A180_Min_Max[ ] = { 0, 255, 0, 255 };
    public final static double clawRightRef_A0_A180_Min_Max[ ] = { 0, 255, 0, 255 };

    // TODO populate real arm dimensions
    public final static double lengthArmOne = 266;
    public final static double lengthArmTwo = 266;
    public final static double lengthClaw = 22;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Gigi_HardwareRobot_Trigonometric() {
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
        clawRight   = hwMap.get( Servo.class, "claw_right" );
        clawLeft    = hwMap.get( Servo.class, "claw_left" );

        armHome();
        clawHome();
    }

    // TODO update all home positions
    public void armHome() {
        moveArmByCoordinatesSteps(
                0,
                33,
                11 );
    }
    public void armFront() {
        moveArmByCoordinatesSteps(
                0,
                222,
                -11 );
    }

    public void armFront_plus_x() {
        moveArmByCoordinatesSteps(
                222,
                222,
                -11 );
    }

    public void armFront_minus_x() {
        moveArmByCoordinatesSteps(
                -222,
                222,
                -11 );
    }

    public void armFront_plus_z() {
        moveArmByCoordinatesSteps(
                0,
                22,
                655 );
    }

    public void clawOpen() {
        moveClawByCoordinates( 66 );
    }

    public void clawClose() {
        moveClawByCoordinates( 55 );
    }

    public void clawHome() {
        moveClawByCoordinates( 111 );
    }

    public void moveArmByCoordinatesSteps( double newX, double newY, double newZ )
    {
        computeCurrentCoordinates();

        double maxChange = 0.0;
        maxChange = Math.max( Math.abs( newX - currentCoordinateX ), maxChange );
        maxChange = Math.max( Math.abs( newY - currentCoordinateY ), maxChange );
        maxChange = Math.max( Math.abs( newZ - currentCoordinateZ ), maxChange );

        int steps = (int)( maxChange / 4 ); // 4 mm per step
        if( steps > 66 ) steps = 66;
        if( steps < 2 ) steps = 2;
        int accel = (int)( (double)steps * 0.2 ); // accelate/decelerate first 20%
        if( accel < 1 ) accel = 1;
        if( accel > steps / 2  ) accel = steps / 2;

        for( int i = 0; i < steps; i++ )
        {
            double stepX = currentCoordinateX + ( ( newX - currentCoordinateX ) * ( i + 1 ) ) / steps;
            double stepY = currentCoordinateY + ( ( newY - currentCoordinateY ) * ( i + 1 ) ) / steps;
            double stepZ = currentCoordinateZ + ( ( newZ - currentCoordinateZ ) * ( i + 1 ) ) / steps;

            moveArmByCoordinates( stepX, stepY, stepZ );

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
            if( ( steps - i ) < accel ){
                extraWait = ( accel - ( steps - i ) ) * 33;
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


    public void moveArmByServoPos( double turretNew, double bottomNew, double topNew, double wristUpDownNew, double wristLeftRightNew)
    {
        turret.setPosition( turretNew );
        bottom.setPosition( bottomNew );
        top.setPosition( topNew );
        wrist.setPosition( wristUpDownNew );
        // TODO add wristLeftRightNew
    }

    public void moveArmByCoordinates( double newX, double newY, double newZ )
    {
        // TODO fix robot dimensions and limits below
        double maxTriangleFlatness = 0.77;
        double closestToTurretY = 66;
        double lowestZ = -66;
        double edgeOfRobotY = 199;
        double heightOfRobotPlatformZ = 111;

        computeCurrentCoordinates();

        if( newY > ( lengthArmOne + lengthArmTwo ) * maxTriangleFlatness ) {
            newY = ( lengthArmOne + lengthArmTwo ) * maxTriangleFlatness;
    }
        if( newY < -( lengthArmOne + lengthArmTwo ) * maxTriangleFlatness ) {
            newY = - ( lengthArmOne + lengthArmTwo ) * maxTriangleFlatness;
        }
        if( newX > ( lengthArmOne + lengthArmTwo ) * maxTriangleFlatness ) {
            newX = ( lengthArmOne + lengthArmTwo ) * maxTriangleFlatness;
        }
        if( newX < -( lengthArmOne + lengthArmTwo ) * maxTriangleFlatness ) {
            newX = - ( lengthArmOne + lengthArmTwo ) * maxTriangleFlatness;
        }
        if( newY < closestToTurretY ){
            newY = closestToTurretY;
        }
        if( newZ < lowestZ ){
            newZ = lowestZ;
        }
        if( newZ > ( lengthArmOne + lengthArmTwo ) * maxTriangleFlatness ){
            newZ = ( lengthArmOne + lengthArmTwo ) * maxTriangleFlatness;
        }

        // check such it does not bump in the robot
        if(  newY < edgeOfRobotY ){
            if( newZ < heightOfRobotPlatformZ ){
                newZ = heightOfRobotPlatformZ;
            }
        }

        double projectionBottomHorizontal = Math.sqrt( newY * newY + newZ * newZ );
        if( projectionBottomHorizontal >  ( lengthArmOne + lengthArmTwo ) * maxTriangleFlatness ){
            projectionBottomHorizontal = ( lengthArmOne + lengthArmTwo ) * maxTriangleFlatness;
        }
        if( projectionBottomHorizontal < closestToTurretY ){
            projectionBottomHorizontal = closestToTurretY;
        }

        double l3 = Math.sqrt( newX * newX  + newZ * newZ );
        double l1 = lengthArmOne;
        double l2 = lengthArmTwo;
        double alphaAngle = Math.asin( Math.abs( newZ ) / l3 ) * 180 / Math.PI;

        double angle2 = Math.acos( ( l1 * l1 + l3 * l3 - l2 * l2 ) / ( 2 * l1 * l3 ) ) * 180 / Math.PI;
        double angle3 = Math.acos( ( l1 * l1 + l2 * l2 - l3 * l3 ) / ( 2 * l1 * l2 ) ) * 180 / Math.PI;
        double angle1 = 180 - angle2 - angle3;

        double newBottomAngle = alphaAngle + angle2;
        if( newZ < 0 ) newBottomAngle = angle2 - alphaAngle;
        double newTopAngle = angle3;

        double newWristAngleH = 0;
        double betaAngle = Math.asin( Math.abs( newX / projectionBottomHorizontal ) ) * 180 / Math.PI;
        newWristAngleH = 90 + betaAngle;
        if( newX > 0 ) newWristAngleH -= 90;

        double newWristAngleV = angle1 - alphaAngle;
        if( newZ < 0 ) newWristAngleV = angle1 + alphaAngle;

        double newTurretAngle = 90 - betaAngle;
        if( newX > 0 ) newTurretAngle += 90;
        if( newTurretAngle < 0 ) newTurretAngle = 0;
        if( newTurretAngle > 180 ) newTurretAngle = 180;

        double newTurretPos = servoPosFromAngle( newTurretAngle,
                turretRef_A0_A180_Min_Max[ 0 ],
                turretRef_A0_A180_Min_Max[ 1 ],
                turretRef_A0_A180_Min_Max[ 2 ],
                turretRef_A0_A180_Min_Max[ 3 ] );
        double newBottomPos = servoPosFromAngle( newBottomAngle,
                bottomRef_A0_A180_Min_Max[ 0 ],
                bottomRef_A0_A180_Min_Max[ 1 ],
                bottomRef_A0_A180_Min_Max[ 2 ],
                bottomRef_A0_A180_Min_Max[ 3 ] );
        double newTopPos = servoPosFromAngle( newTopAngle,
                topRef_A0_A180_Min_Max[ 0 ],
                topRef_A0_A180_Min_Max[ 1 ],
                topRef_A0_A180_Min_Max[ 2 ],
                topRef_A0_A180_Min_Max[ 3 ]  );
        double newWristUpDownPos = servoPosFromAngle( newWristAngleH,
                wristUpDownRef_A0_A180_Min_Max[ 0 ],
                wristUpDownRef_A0_A180_Min_Max[ 1 ],
                wristUpDownRef_A0_A180_Min_Max[ 2 ],
                wristUpDownRef_A0_A180_Min_Max[ 3 ]  );
        double newWristLeftRightPos = servoPosFromAngle( newWristAngleV,
                wristLeftRightRef_A0_A180_Min_Max[ 0 ],
                wristLeftRightRef_A0_A180_Min_Max[ 1 ],
                wristLeftRightRef_A0_A180_Min_Max[ 2 ],
                wristLeftRightRef_A0_A180_Min_Max[ 3 ] );

        moveArmByServoPos( newTurretPos, newBottomPos, newTopPos, newWristUpDownPos, newWristLeftRightPos );
    }

    public double servoPosFromAngle( double angle, double refA0, double refA180, double refMin, double refMax )
    {
        double pos = 0;
        if( refA180 > refA0 ){
            pos = ( angle - refA0 ) / 180 * ( refA180 - refA0 );
            if( pos < refMin ) pos = refMin;
            if( pos > refMax ) pos = refMax;
        }
        if( refA0 > refA180 ){
            pos = 255 - ( angle - refA0 ) / 180 * (  refA0 - refA180 );
            if( pos > refMin ) pos = refMin;
            if( pos < refMax ) pos = refMax;
        }
        if( pos < refMin ) pos = refMin;
        if( pos > refMax ) pos = refMax;
        if( pos < 0 ) pos = 0;
        if( pos > 1 ) pos = 1;

        return pos;
    }

    public double servoAngleFromPos( double pos, double refA0, double refA180, double refMin, double refMax )
    {
        double angle = 0;

        if( refA0 < refA180 ) {
            angle = ( pos / ( refA180 - refA0 ) ) * 180;
            angle += refA0;
        }
        if( refA0 > refA180 ) {
            angle = ( ( refA0 - pos ) / ( refA0 - refA180 ) ) * 180;
            angle += refA0;
        }
        if( angle < 0 ) angle = 0;
        if( angle > 180 ) angle = 180;

        return angle;
    }

    public void moveClawByCoordinates( double opening )
    {
        if( opening < 2 * lengthClaw * 0.2 )opening = 2 * lengthClaw * 0.2;
        if( opening > 2 * lengthClaw * 0.8 )opening = 2 * lengthClaw * 0.8;

        double angleLeft = Math.asin( ( opening / 2 ) / lengthClaw );
        double angleRigh = Math.asin( ( opening / 2 ) / lengthClaw );

        clawLeft.setPosition( servoPosFromAngle( angleLeft,
                clawLeftRef_A0_A180_Min_Max[ 0 ],
                clawLeftRef_A0_A180_Min_Max[ 1 ],
                clawLeftRef_A0_A180_Min_Max[ 2 ],
                clawLeftRef_A0_A180_Min_Max[ 3 ] ) );
        clawRight.setPosition( servoPosFromAngle( angleRigh,
                clawRightRef_A0_A180_Min_Max[ 0 ],
                clawRightRef_A0_A180_Min_Max[ 1 ],
                clawRightRef_A0_A180_Min_Max[ 2 ],
                clawRightRef_A0_A180_Min_Max[ 3 ] ) );
    }

    public void computeCurrentCoordinates()
    {
        double turretAngle = servoAngleFromPos( turret.getPosition(),
                turretRef_A0_A180_Min_Max[ 0 ],
                turretRef_A0_A180_Min_Max[ 1 ],
                turretRef_A0_A180_Min_Max[ 2 ],
                turretRef_A0_A180_Min_Max[ 3 ] );
        double bottomAngle = servoAngleFromPos( bottom.getPosition(),
                bottomRef_A0_A180_Min_Max[ 0 ],
                bottomRef_A0_A180_Min_Max[ 1 ],
                bottomRef_A0_A180_Min_Max[ 2 ],
                bottomRef_A0_A180_Min_Max[ 3 ] );
        double topAngle = servoAngleFromPos( top.getPosition(),
                topRef_A0_A180_Min_Max[ 0 ],
                topRef_A0_A180_Min_Max[ 1 ],
                topRef_A0_A180_Min_Max[ 2 ],
                topRef_A0_A180_Min_Max[ 3 ] );

        // compute the projection on the horizontal plane
        double projectionBottomHorizontal = 0;
        if( bottomAngle < 90 ) {
            projectionBottomHorizontal += lengthArmOne * Math.cos(bottomAngle * Math.PI / 180);
            projectionBottomHorizontal += lengthArmTwo * Math.sin((topAngle - (90 - bottomAngle)) * Math.PI / 180);
        }
        if( bottomAngle > 90 ) {
            projectionBottomHorizontal -= lengthArmOne * Math.cos((bottomAngle-90) * Math.PI / 180);
            projectionBottomHorizontal += lengthArmTwo * Math.sin((topAngle - (90 - bottomAngle)) * Math.PI / 180);
        }

        // compute the projection on front vertical
        double projectionFrontVertical = 0;
        projectionFrontVertical += lengthArmOne * Math.sin( bottomAngle * Math.PI / 180 );
        projectionFrontVertical -= lengthArmTwo * Math.cos( ( topAngle - ( 90 - bottomAngle ) ) * Math.PI / 180 );

        currentCoordinateZ = projectionFrontVertical;

        // compute the projections on bottom horizontal
        currentCoordinateX = -projectionBottomHorizontal * Math.cos( ( turretAngle ) * Math.PI / 180 );
        if( turretAngle > 90 )currentCoordinateY = projectionBottomHorizontal * Math.sin( ( turretAngle - 90 ) * Math.PI / 180 );
        if( turretAngle < 90 )currentCoordinateY = projectionBottomHorizontal * Math.sin( ( turretAngle ) * Math.PI / 180 );
    }
}
