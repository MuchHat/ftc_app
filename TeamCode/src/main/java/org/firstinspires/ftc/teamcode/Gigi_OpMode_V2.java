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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file illustrates the concept of driving a path based on time.
 */

@TeleOp(name="OpMode V2", group="Gigi")
// @Disabled
public class Gigi_OpMode_V2 extends OpMode{

    public  Gigi_Hardware_V2   robot         = new Gigi_Hardware_V2();
    public  ArmController      armController = new ArmController();
    public  ElapsedTime        runtime       = new ElapsedTime();

    public  double             turretControl = 0;
    public  double             baseControl   = 0;
    public  double             elbowControl  = 0;
    public  double             wristControl  = 0;
    public  double             clawControlL  = 0;
    public  double             clawControlR  = 0;
    public  boolean            initialized   = false;

    public double t_pi = 0;
    public double b_pi = 0;
    public double e_pi = 0;

    public double r = 0;
    public double teta = 0;
    public double phi = 0;

    public double x = 0;
    public double y = 0;
    public double z = 0;

    public double rr = 0;
    public double tteta = 0;
    public double tphi = 0;

    public double tt = 0;
    public double bb = 0;
    public double ee = 0;


    @Override
    public void init() {
         /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init( hardwareMap );
        robot.leftDrive.setPower( 0 );
        robot.rightDrive.setPower( 0 );
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

    public void testServos( double ts, double bs, double es ) {

        // use polar coordinates
        // all angles are 0 to PI
        // https://en.wikipedia.org/wiki/Spherical_coordinate_system

        t_pi = ts * Math.PI;
        b_pi = bs * Math.PI;
        e_pi = es * Math.PI;

        teta = Math.PI - t_pi;
        phi = b_pi - Math.PI / 2;

        Triangle elbowTriangle = new Triangle();
        elbowTriangle.solve_SSA(240, 240, e_pi);

        r = elbowTriangle.l3;

        x = r * Math.sin(phi) * Math.cos(teta);
        y = r * Math.sin(phi) * Math.sin(teta);
        z = r * Math.cos(phi);
    }

    public void testXYZ( double xx, double yy, double zz ) {

        rr = Math.sqrt( x * x + y * y + z * z );

        tphi = Math.acos( z / r );
        tteta = Math.atan( y / x );

        Triangle elbowTriangle = new Triangle();
        elbowTriangle.solve_SSS( 240, 240, rr );

        tt = Math.PI - tteta;
        bb =  Math.PI / 2 + tphi;
        ee =  elbowTriangle.a3;
    }

    @Override
    public void loop() {

        double crrLoopTime = runtime.milliseconds();
        runtime.reset();

        if ( !initialized ) {
            robot._turret.setPosition( 166 / 255 );
            robot._base.setPosition( 30 / 255 );
            robot._elbow.setPosition( 80 / 255 );
            robot._wrist.setPosition( 80 / 255 );
            robot._leftClaw.setPosition( 0 / 255 );
            robot._rightClaw.setPosition( 140 / 255 );

            turretControl = robot._turret.getPosition();
            baseControl   = robot._base.getPosition();
            elbowControl  = robot._elbow.getPosition();
            wristControl  = robot._wrist.getPosition();
            clawControlL  = robot._leftClaw.getPosition();
            clawControlR  = robot._rightClaw.getPosition();
            initialized   = true;
        }
        else if( initialized ){
            turretControl = robot._turret.getPosition();
            baseControl   = robot._base.getPosition();
            elbowControl  = robot._elbow.getPosition();
            wristControl  = robot._wrist.getPosition();
            clawControlL  = robot._leftClaw.getPosition();
            clawControlR  = robot._rightClaw.getPosition();
        }

        // armController.startLoop( t, b, e );
        testServos( turretControl, baseControl, elbowControl );
        testXYZ( x, y, z );

        //String atDestinationStr = new String( "moving->" );
        //if( armController.atDestination ){
        //    atDestinationStr = "stopped";
        //}

        telemetry.addData("t_pi","%.2fpi", t_pi );
        telemetry.addData("b_pi","%.2fpi", b_pi );
        telemetry.addData("e_pi", "%.2fpi", e_pi );
        telemetry.addData("r", "%.2fmm", r );
        telemetry.addData("teta","%.2fpi",teta );
        telemetry.addData("phi","%.2fpi", phi );
        telemetry.addData("X axis","%.2fmm", x );
        telemetry.addData("Y axis","%.2fmm", y );
        telemetry.addData("Z axis","%.2fmm", z );
        telemetry.addData("tt","%.2fpi", rr );
        telemetry.addData("bb","%.2fpi", bb );
        telemetry.addData("ee","%.2fpi", ee );
        telemetry.update();

        // control: TURRET
        {
            turretControl += gamepad1.left_stick_x * 0.0003 * crrLoopTime;
            turretControl = Range.clip(turretControl, 0.15, 0.95 );
            robot._turret.setPosition(turretControl);
        }
        // control: BASE
        {
            baseControl += gamepad1.right_stick_y * 0.0003 * crrLoopTime;
            baseControl = Range.clip(baseControl, 0.15, 0.95 );
            robot._base.setPosition(baseControl);
        }
        // control: ELBOW
        {
            elbowControl += gamepad1.left_stick_y * 0.0003 * crrLoopTime;
            elbowControl = Range.clip(elbowControl, 0.15, 0.95 );
            robot._elbow.setPosition(elbowControl);
        }
        // control: WRIST
        {
            wristControl += gamepad1.right_stick_x * 0.0003 * crrLoopTime;
            wristControl = Range.clip(wristControl, 0.15, 0.95 );
            robot._wrist.setPosition(wristControl);
        }
        // control: CLAW OPEN
        {
            clawControlR += gamepad1.right_trigger * 0.0003 * crrLoopTime;
            clawControlL -= gamepad1.right_trigger * 0.0003 * crrLoopTime;
            clawControlR = Range.clip(clawControlR, 0.15, 0.95 );
            clawControlL = Range.clip(clawControlL, 0.15, 0.95 );
            robot._rightClaw.setPosition(clawControlR);
            robot._leftClaw.setPosition(clawControlL);
        }
        // control: CLAW CLOSE
        {
            clawControlR -= gamepad1.right_trigger * 0.0003 * crrLoopTime;
            clawControlL += gamepad1.right_trigger * 0.0003 * crrLoopTime;
            clawControlR = Range.clip(clawControlR, 0.15, 0.95 );
            clawControlL = Range.clip(clawControlL, 0.15, 0.95 );
            robot._rightClaw.setPosition(clawControlR);
            robot._leftClaw.setPosition(clawControlL);
        }
        // control: HOME
        {
            if ( gamepad1.a ) {
                robot._turret.setPosition( 166 / 255 );
                robot._base.setPosition( 30 / 255 );
                robot._elbow.setPosition( 80 / 255 );
                robot._wrist.setPosition( 80 / 255 );
                robot._leftClaw.setPosition( 0 / 255 );
                robot._rightClaw.setPosition( 140 / 255 );
            }
        }
        // control: LEFT RIGH MM
        {
            if ( gamepad1.dpad_up ) {
                armController.moveIncremental( 0, 5, 0 );
            }
        }
        // control: LEFT RIGH MM
        {
            if ( gamepad1.dpad_down ) {
                armController.moveIncremental( 0, -5, 0 );
            }
        }
        // control: LEFT RIGH MM
        {
            if ( gamepad1.dpad_right ) {
                armController.moveIncremental( 5, 0, 0 );
            }
        }
        // control: LEFT RIGH MM
        {
            if ( gamepad1.dpad_left ) {
                armController.moveIncremental( -5, 0, 0 );
            }
        }
        // control: LEFT RIGH MM
        {
            if ( gamepad1.right_bumper ) {
                armController.moveIncremental( 0, 0, 5 );
            }
        }
        // control: LEFT RIGH MM
        {
            if ( gamepad1.left_bumper ) {
                armController.moveIncremental( 0, 0, -5 );
            }
        }
        // control: FRONT
        {
            if ( gamepad1.a ) {
                armController.moveToPositionFront();
            }
        }
        // control: ZERO
        {
            if ( gamepad1.x ) {
                armController.moveToPositionZero();
            }
        }
        // control: HOME
        {
            if ( gamepad1.y ) {
                armController.moveToPositionHome();
            }
        }

       // set the servos per the ARM controller
       // armController.endLoop( crrLoopTime );

        /* TODO enable below later
        robot._turret.setPosition( armController.next.turretAngle.angleServo );
        robot._base.setPosition( armController.next.baseAngle.angleServo );
        robot._elbow.setPosition( armController.next.elbowAngle.angleServo );
        robot._wrist.setPosition( armController.next.clawVerticalAngle.angleServo );

        robot._leftClaw.setPosition( armController.next.clawOpeningAngle.angleServo );
        robot._rightClaw.setPosition( 140 - armController.next.clawOpeningAngle.angleServo );
        TODO END */
/*
        telemetry.addData("turret\t  ", "%.2f  ctrl  %.3f ", robot._turret.getPosition(), turretControl);
        telemetry.addData("  base\t  ", "%.2f  ctrl  %.3f ", robot._base.getPosition(), baseControl);
        telemetry.addData(" elbow\t  ", "%.2f  ctrl  %.3f ", robot._elbow.getPosition(), elbowControl);
        telemetry.addData(" wrist\t  ", "%.2f  ctrl  %.3f ", robot._wrist.getPosition(), wristControl);
        telemetry.addData("claw_l\t  ", "%.2f  ctrl  %.3f ", robot._leftClaw.getPosition(), clawControlL);
        telemetry.addData("claw_r\t  ", "%.2f  ctrl  %.3f ", robot._rightClaw.getPosition(), clawControlR);
        telemetry.update();
*/
    }
    /*
    * Code to run ONCE after the driver hits STOP
    */
    @Override
    public void stop() {
    }
}
