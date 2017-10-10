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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file illustrates the concept of driving a path based on time.
 */

@TeleOp(name = "Test Servos V2", group = "Team")
// @Disabled
public class Team_TestServos_V2 extends LinearOpMode {

    public Team_Hardware_V2 robot = new Team_Hardware_V2();
    public ElapsedTime runtime = new ElapsedTime();
    //Declare arm atuff
    public double baseControl = 0.5;
    public double elbowControl = 0.5;
    public double leftClawControl = 0.5;
    public double rightClawControl = 0.5;

    ElapsedTime timer = new ElapsedTime();
    double servoDefaultSpeed = 0.00033; // 0.33 servo angle per sec

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.base.setPosition(baseControl);
        robot.elbow.setPosition(elbowControl);
        robot.leftClaw.setPosition(leftClawControl);
        robot.rightClaw.setPosition(rightClawControl);

        waitForStart();

        while (opModeIsActive()) {

            double crrLoopTime = runtime.nanoseconds() / 1000000; // covert to millis
            runtime.reset();

            baseControl = robot.base.getPosition();
            elbowControl = robot.elbow.getPosition();
            leftClawControl = robot.leftClaw.getPosition();
            rightClawControl = robot.rightClaw.getPosition();

            telemetry.addData("base", "%.0f%%", baseControl * 100);
            telemetry.addData("elbow", "%.0f%%", elbowControl * 100 );
            telemetry.addData("left claw", "%.0f%%", leftClawControl * 100);
            telemetry.addData("right claw", "%.0f%%", rightClawControl * 100);
            telemetry.update();

            // control: BASE
            if (gamepad1.left_stick_y != 0) {
                baseControl += gamepad1.left_stick_y * servoDefaultSpeed * crrLoopTime;
                setServos();
            }
            // control: ELBOW
            if (gamepad1.right_stick_y != 0) {
                elbowControl += gamepad1.right_stick_y * servoDefaultSpeed * crrLoopTime;
                setServos();
            }
            // control: CLAW CLOSE
            if (gamepad1.left_trigger != 0) {
                leftClawControl += gamepad1.left_stick_x * servoDefaultSpeed * crrLoopTime;
                setServos();
            }
            // control: CLAW OPEN
            if (gamepad1.right_trigger != 0) {
                rightClawControl += gamepad1.right_stick_x * servoDefaultSpeed * crrLoopTime;
                setServos();
            }
        }
    }

    public void setServos() {

        robot.elbow.setPosition(elbowControl);
        robot.base.setPosition(baseControl);
        robot.leftClaw.setPosition(leftClawControl);
        robot.rightClaw.setPosition(rightClawControl);
    }
}

