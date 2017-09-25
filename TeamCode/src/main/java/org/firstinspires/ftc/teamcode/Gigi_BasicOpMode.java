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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Gigi: Iterative OpMode", group="Gigi")
@Disabled
public class Gigi_BasicOpMode extends OpMode
{
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;

    public Servo turret      = null;
    public Servo    base      = null;
    public Servo    elbow         = null;
    public Servo    wrist       = null;
    public Servo    clawRight   = null;
    public Servo    clawLeft    = null;

    public double currentCoordinateX = 0;
    public double currentCoordinateY = 0;
    public double currentCoordinateZ = 0;

    public double turretLastSet = 0;
    public double baseLastSet = 0;
    public double elbowLastSet = 0;
    public double wristLastSet = 0;
    public double lastGamePadRead = 0;
    public double newGamePadSessionStart = 0;

    // alpha is number between 0 and 180 indicating the pos of min for the servo
    // min, max, home are numbers between 0 to 255

    // TODO populate servo info
    public final static double turretRef_A0_A180_Min_Max[ ] = { 40, 300, 40, 255 };
    public final static double baseRef_A0_A180_Min_Max[ ] = { -10, 235, 0, 255 };
    public final static double elbowRef_A0_A180_Min_Max[ ] = { 30, 320, 40, 255 };
    public final static double wristUpDownRef_A0_A180_Min_Max[ ] = { -20, 240, 0, 240 };
    public final static double wristLeftRightRef_A0_A180_Min_Max[ ] = { 0, 255, 0, 255 };
    public final static double clawLeftRef_A0_A180_Min_Max[ ] = { -10, 120, 0, 120 };
    public final static double clawRightRef_A0_A180_Min_Max[ ] = { 140, 280, 140, 255 };

    // TODO populate real arm dimensions
    public final static double lengthArmOne = 266;
    public final static double lengthArmTwo = 266;
    public final static double lengthClaw = 22;

    /* Local OpMode members. */
    private ElapsedTime period  = new ElapsedTime();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Start init");

        // Define and Initialize Motors
        leftDrive  = hardwareMap.get( DcMotor.class, "Motor_Left" );
        rightDrive = hardwareMap.get( DcMotor.class, "Motor_Right" );
        leftDrive.setDirection( DcMotor.Direction.FORWARD );
        leftDrive.setDirection( DcMotor.Direction.FORWARD );

        // Set all motors to zero power
        leftDrive.setPower( 0 );
        rightDrive.setPower( 0 );

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        rightDrive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        turret = hardwareMap.get(Servo.class, "Turret");
        base = hardwareMap.get(Servo.class, "Base");
        elbow = hardwareMap.get(Servo.class, "Elbow");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        clawLeft = hardwareMap.get(Servo.class, "Claw_Left");
        clawRight = hardwareMap.get(Servo.class, "Claw_Right");

        armHome();
        clawHome();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "End init");
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

    public double getTurret()
    {
        return turretLastSet;
    }
    public double getBase()
    {
        return baseLastSet;
    }
    public double getElbow()
    {
        return elbowLastSet;
    }
    public double getWrist()
    {
        return wristLastSet;
    }

    public void moveArmByServoPos( double turretNew, double baseNew, double elbowNew, double wristUpDownNew, double wristLeftRightNew)
    {

        // turret.setPosition( turretNew );
        // base.setPosition( baseNew  );
        // elbow.setPosition( elbowNew  );
        // wrist.setPosition( wristUpDownNew );
        // TODO add wristLeftRightNew

        turretLastSet = turretNew;
        baseLastSet = baseNew;
        elbowLastSet = elbowNew;
        wristLastSet = wristUpDownNew;

        telemetry.addData( "RUNNING  at ", "%5.2f", (double)System.currentTimeMillis() / 1000 );
        telemetry.addData( "TURRET   at ", "%5.2f", turret.getPosition() );
        telemetry.addData( "BASE     at ", "%5.2f", base.getPosition() );
        telemetry.addData( "ELBOW    at ", "%5.2f", elbow.getPosition() );
        telemetry.addData( "WRIST    at ", "%5.2f", wrist.getPosition() );
        telemetry.update();
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
                baseRef_A0_A180_Min_Max[ 0 ],
                baseRef_A0_A180_Min_Max[ 1 ],
                baseRef_A0_A180_Min_Max[ 2 ],
                baseRef_A0_A180_Min_Max[ 3 ] );
        double newTopPos = servoPosFromAngle( newTopAngle,
                elbowRef_A0_A180_Min_Max[ 0 ],
                elbowRef_A0_A180_Min_Max[ 1 ],
                elbowRef_A0_A180_Min_Max[ 2 ],
                elbowRef_A0_A180_Min_Max[ 3 ]  );
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
        pos /= 255;
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

        //clawLeft.setPosition( servoPosFromAngle( angleLeft,
        //       clawLeftRef_A0_A180_Min_Max[ 0 ],
        //       clawLeftRef_A0_A180_Min_Max[ 1 ],
        //       clawLeftRef_A0_A180_Min_Max[ 2 ],
        //       clawLeftRef_A0_A180_Min_Max[ 3 ] ) );
        //clawRight.setPosition( servoPosFromAngle( angleRigh,
        //      clawRightRef_A0_A180_Min_Max[ 0 ],
        //      clawRightRef_A0_A180_Min_Max[ 1 ],
        //      clawRightRef_A0_A180_Min_Max[ 2 ],
        //      clawRightRef_A0_A180_Min_Max[ 3 ] ) );
    }

    public void computeCurrentCoordinates()
    {
        double turretAngle = servoAngleFromPos( turret.getPosition(),
                turretRef_A0_A180_Min_Max[ 0 ],
                turretRef_A0_A180_Min_Max[ 1 ],
                turretRef_A0_A180_Min_Max[ 2 ],
                turretRef_A0_A180_Min_Max[ 3 ] );
        double baseAngle = servoAngleFromPos( base.getPosition(),
                baseRef_A0_A180_Min_Max[ 0 ],
                baseRef_A0_A180_Min_Max[ 1 ],
                baseRef_A0_A180_Min_Max[ 2 ],
                baseRef_A0_A180_Min_Max[ 3 ] );
        double elbowAngle = servoAngleFromPos( elbow.getPosition(),
                elbowRef_A0_A180_Min_Max[ 0 ],
                elbowRef_A0_A180_Min_Max[ 1 ],
                elbowRef_A0_A180_Min_Max[ 2 ],
                elbowRef_A0_A180_Min_Max[ 3 ] );

        // compute the projection on the horizontal plane
        double projectionBottomHorizontal = 0;
        if( baseAngle < 90 ) {
            projectionBottomHorizontal += lengthArmOne * Math.cos(baseAngle * Math.PI / 180);
            projectionBottomHorizontal += lengthArmTwo * Math.sin((elbowAngle - (90 - baseAngle)) * Math.PI / 180);
        }
        if( baseAngle > 90 ) {
            projectionBottomHorizontal -= lengthArmOne * Math.cos((baseAngle-90) * Math.PI / 180);
            projectionBottomHorizontal += lengthArmTwo * Math.sin((elbowAngle - (90 - baseAngle)) * Math.PI / 180);
        }

        // compute the projection on front vertical
        double projectionFrontVertical = 0;
        projectionFrontVertical += lengthArmOne * Math.sin( baseAngle * Math.PI / 180 );
        projectionFrontVertical -= lengthArmTwo * Math.cos( ( elbowAngle - ( 90 - baseAngle ) ) * Math.PI / 180 );

        currentCoordinateZ = projectionFrontVertical;

        // compute the projections on base horizontal
        currentCoordinateX = -projectionBottomHorizontal * Math.cos( ( turretAngle ) * Math.PI / 180 );
        if( turretAngle > 90 )currentCoordinateY = projectionBottomHorizontal * Math.sin( ( turretAngle - 90 ) * Math.PI / 180 );
        if( turretAngle < 90 )currentCoordinateY = projectionBottomHorizontal * Math.sin( ( turretAngle ) * Math.PI / 180 );
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        init();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData( "Say", "Started" );    //
        telemetry.update();

        // run until the end of the match (driver presses STOP)


        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.
        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;

        if( !(  ( drive == 0.0 ) &&
                ( turn == 0.0 ) &&
                ( leftDrive.getPower() == 0 ) &&
                ( rightDrive.getPower() == 0 ) ) ){

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
            leftDrive.setPower(left);
            rightDrive.setPower(right);
        }

        if (gamepad1.a) {
            telemetry.addData("RobotMove", "Arm Front");
            telemetry.update();
            armFront();
        }

        if (gamepad1.x) {
            telemetry.addData("RobotMove", "Arm Front Mins X");
            telemetry.update();
            armFront_minus_x();
        }

        if (gamepad1.b) {
            telemetry.addData("RobotMove", "Arm Front Plus X");    //
            telemetry.update();
            armFront_plus_x();
        }

        if (gamepad1.y) {
            telemetry.addData("RobotMove", "Arm Front Plus Z");    //
            telemetry.update();
            armFront_plus_z();
        }

        if (gamepad1.right_bumper) {
            telemetry.addData("RobotMove", "Claw Open");    //
            telemetry.update();
            clawOpen();
        }

        if (gamepad1.left_bumper) {
            telemetry.addData("RobotMove", "Claw Close");    //
            telemetry.update();
            clawClose();
        }

        if (gamepad1.back) {
            armHome();
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

            computeCurrentCoordinates();

            // time elapsed is in millis
            // speed is in mm/sec
            double changeX = ( millisElapsed * speedX ) / 1000;
            double changeY = ( millisElapsed * speedY ) / 1000;
            double changeZ = ( millisElapsed * speedZ ) / 1000;

            double newX = currentCoordinateX + changeX;
            double newY = currentCoordinateY + changeY;
            double newZ = currentCoordinateZ + changeZ;

            moveArmByCoordinates( newX, newY, newZ );

        }

        telemetry.addData( "RUNNING  at ", "%5.2f", (double)System.currentTimeMillis() / 1000 );
        telemetry.addData( "TURRET   at ", "%5.2f", getTurret() );
        telemetry.addData( "BASE     at ", "%5.2f", getBase());
        telemetry.addData( "ELBOW    at ", "%5.2f", getElbow() );
        telemetry.addData( "WRIST    at ", "%5.2f", getWrist() );
        telemetry.update();

        // Pace this loop so jaw action is reasonable speed.
        // sleep( 50 );
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        //TODO should bring to homepos
    }

}
