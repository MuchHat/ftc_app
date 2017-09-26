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

/**
 * This is a robot hardware class NOT an opmode.
 *
 */
public class Gigi_Hardware_V2 {
    /* Public OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    public SafeServo leftClaw = null;
    public SafeServo rightClaw = null;
    public SafeServo turret = null;
    public SafeServo base = null;
    public SafeServo elbow = null;
    public SafeServo wrist = null;

    public Servo _leftClaw = null;
    public Servo _rightClaw = null;
    public Servo _turret = null;
    public Servo _base = null;
    public Servo _elbow = null;
    public Servo _wrist = null;

    public double robot_arm_len = 222; // arm lenght
    public double robot_height = 66;
    public double robot_turret_to_edge = 111;

    /* local OpMode members. */
    public HardwareMap hwMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public Gigi_Hardware_V2() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //****** MOTORS *********
        leftDrive = hwMap.get(DcMotor.class, "Motor_Left");
        rightDrive = hwMap.get(DcMotor.class, "Motor_Right");

        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        _turret = hwMap.get(Servo.class, "Turret");
        _base = hwMap.get(Servo.class, "Base");
        _elbow = hwMap.get(Servo.class, "Elbow");
        _wrist = hwMap.get(Servo.class, "Wrist");
        _leftClaw = hwMap.get(Servo.class, "Claw_Left");
        _rightClaw = hwMap.get(Servo.class, "Claw_Right");

        //****** SERVOS *********
        turret = new SafeServo();
        base = new SafeServo();
        elbow = new SafeServo();
        wrist = new SafeServo();
        leftClaw = new SafeServo();
        rightClaw = new SafeServo();

        turret.init(_turret);
        base.init(_base);
        elbow.init(_elbow);
        wrist.init(_wrist);
        leftClaw.init(_leftClaw);
        rightClaw.init(_rightClaw);

        turret.configLimits(4, 255, 40, 295);
        base.configLimits(0, 255, -10, 245);
        elbow.configLimits(40, 255, 30, 330);
        wrist.configLimits(0, 240, -20, 245);
        leftClaw.configLimits(0, 120, -10, 120);
        rightClaw.configLimits(140, 280, 140, 280);

        turret.configHome(166);
        base.configHome(30);
        elbow.configHome(80);
        wrist.configHome(80);
        leftClaw.configHome(0);
        rightClaw.configHome(140);
    }

    double zMoveToBase( double d ) {

        double e1 = elbow.getAngle();
        double b1 = base.getAngle();

        Triangle t1 = new Triangle();
        Triangle t2 = new Triangle();
        Triangle t3 = new Triangle();

        t1.resolve_SSA( robot_arm_len, robot_arm_len, e1 );
        t2.resolve_SSA( t1.l3, d, Math.PI - b1  );
        t3.resolve_SSS( robot_arm_len, robot_arm_len, t2.l3 );

        return t2.a2 - b1;
    }

    double zMoveToElbow( double d ) {

        double e1 = elbow.getAngle();
        double b1 = base.getAngle();

        Triangle t1 = new Triangle();
        Triangle t2 = new Triangle();
        Triangle t3 = new Triangle();

        t1.resolve_SSA( robot_arm_len, robot_arm_len, e1 );
        t2.resolve_SSA( t1.l3, d, Math.PI - b1  );
        t3.resolve_SSS( robot_arm_len, robot_arm_len, t2.l3 );

        return t1.a3 - e1;
    }

    double zMoveToWrist( double d ) {

        double e1 = elbow.getAngle();
        double b1 = base.getAngle();

        Triangle t1 = new Triangle();
        Triangle t2 = new Triangle();
        Triangle t3 = new Triangle();

        t1.resolve_SSA( robot_arm_len, robot_arm_len, e1 );
        t2.resolve_SSA( t1.l3, d, Math.PI - b1  );
        t3.resolve_SSS( robot_arm_len, robot_arm_len, t2.l3 );

        return t1.a2 - t2.a2;
    }
    
    /********************* BELOW ARE NOT COMPLETE *******************/

    double yMoveToBase( double d )  {

        double e1 = elbow.getAngle();
        double b1 = base.getAngle();

        Triangle t1 = new Triangle();
        Triangle t2 = new Triangle();
        Triangle t3 = new Triangle();

        t1.resolve_SSA( robot_arm_len, robot_arm_len, e1 );
        t2.resolve_SSA( t1.l3, d, Math.PI - b1  );
        t3.resolve_SSS( robot_arm_len, robot_arm_len, t2.l3 );

        return -t2.a2;
    }

    double yMoveToElbow( double d ) {

        double e1 = elbow.getAngle();
        double b1 = base.getAngle();

        Triangle t1 = new Triangle();
        Triangle t2 = new Triangle();
        Triangle t3 = new Triangle();

        t1.resolve_SSA( robot_arm_len, robot_arm_len, e1 );
        t2.resolve_SSA( t1.l3, d, Math.PI - b1  );
        t3.resolve_SSS( robot_arm_len, robot_arm_len, t2.l3 );

        return t1.a3 - e1;
    }

    double yMoveToWrist( double d ) {

        double e1 = elbow.getAngle();
        double b1 = base.getAngle();

        Triangle t1 = new Triangle();
        Triangle t2 = new Triangle();
        Triangle t3 = new Triangle();

        t1.resolve_SSA( robot_arm_len, robot_arm_len, e1 );
        t2.resolve_SSA( t1.l3, d, Math.PI - b1  );
        t3.resolve_SSS( robot_arm_len, robot_arm_len, t2.l3 );

        return t1.a2 - t2.a2;
    }

    double yMoveToTurret( double d ) {
        return 0;
    }

    double xMoveToBase( double d ) {
        return 0;
    }

    double xMoveToElbow( double d ) {
        return 0;
    }

    double xMoveToWrist( double d ) {
        return 0;
    }

    double xMoveToTurret( double d ) {
        return 0;
    }
}
