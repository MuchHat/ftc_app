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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// ************************** OP FOR TESTING THE SERVOS ******************************************//

@TeleOp(name = "Test All V9", group = "Test")
//@Disabled
public class Test_All_V9 extends LinearOpMode {

    // ************************** VARIABLES ******************************************************//

    private Team_Hardware_V9 robot = new Team_Hardware_V9();

    private ElapsedTime runtimeLoop = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();

    private double baseControl = 0.5;
    private double elbowControl = 0.0;
    private double leftClawControl = 0.5;
    private double rightClawControl = 0.5;
    private double driveDistanceControl = 0;
    private double driveDistanceForOnePointZero = 1000; //distance at full stick
    private double servoDefaultSpeed = 0.000066 * 2; // 0.33 servo angle per sec
    private double totalDistance = 0;


    // ************************** OP LOOP ********************************************************//

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.log().add("calibrating gyro, do not move");
        telemetry.update();
        sleep(444);
        robot.modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        runtimeLoop.reset();
        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating gyro", "%s",
                    Math.round(runtimeLoop.seconds()) % 2 == 0 ? "..  " : "   ..");
            telemetry.update();
            sleep(66);
        }
        runtimeLoop.reset();
        telemetry.log().clear();

        robot.moveArmPosZero();
        robot.stopRobot();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            double crrLoopTime = runtimeLoop.nanoseconds() / 1000000; // covert to millis
            runtimeLoop.reset();

            robot.colorSensor.enableLed(true);

            telemetry.addData("total distance", "%.0f", totalDistance);
            telemetry.addData("left drive pos", "%.0f", (double) robot.leftDrive.getCurrentPosition());
            telemetry.addData("right drive pos", "%.0f", (double) robot.rightDrive.getCurrentPosition());
            telemetry.addData("left drive back pos", "%.0f", (double) robot.leftDriveBack.getCurrentPosition());
            telemetry.addData("right drive back pos", "%.0f", (double) robot.rightDriveBack.getCurrentPosition());

            telemetry.addData("base", "%.0f%%", robot.base.getPosition() * 100);
            telemetry.addData("elbow", "%.0f%%", robot.elbow.getPosition() * 100);
            telemetry.addData("left claw", "%.0f%%", robot.leftClaw.getPosition() * 100);
            telemetry.addData("right claw", "%.0f%%", robot.rightClaw.getPosition() * 100);

            telemetry.addData("color sensor red", "%.2f", (double) robot.colorSensor.red());
            telemetry.addData("color sensor green", "%.2f", (double) robot.colorSensor.green());
            telemetry.addData("color sensor blue", "%.2f", (double) robot.colorSensor.blue());
            telemetry.addData("color sensor alpha", "%.2f", (double) robot.colorSensor.alpha());

            telemetry.addData("switch top", "%.2f", robot.topSwitch.getState() ? 1.0 : 0.0);
            telemetry.addData("switch bottom", "%.2f", robot.bottomSwitch.getState() ? 1.0 : 0.0);

            telemetry.addData("crr heading", "%.2fdeg", (double) robot.modernRoboticsI2cGyro.getHeading());

            telemetry.update();

            // control: BASE
            if (gamepad1.left_stick_y != 0) {
                robot.baseControl += gamepad1.left_stick_y * servoDefaultSpeed * crrLoopTime;
                robot.setServos();
            }
            // control: ELBOW
            if (gamepad1.right_stick_y != 0) {
                robot.elbowControl += gamepad1.right_stick_y * servoDefaultSpeed * crrLoopTime;
                robot.setServos();
            }
            // control: LEFT CLAW
            if (gamepad1.left_stick_x != 0) {
                robot.leftClawControl += gamepad1.left_stick_x * servoDefaultSpeed * crrLoopTime;
                robot.setServos();
            }
            // control: RIGHT CLAW
            if (gamepad1.right_stick_x != 0) {
                robot.rightClawControl += gamepad1.right_stick_x * servoDefaultSpeed * crrLoopTime * -1;
                robot.setServos();
            }
            // control: RIGHT_TRIGGER
            if (Math.abs(gamepad1.right_trigger) > 0.33) {
                driveDistanceControl = 100;
                robot.move(driveDistanceControl);
                totalDistance += driveDistanceControl;
            }
            // control: LEFT_TRIGGER
            if (Math.abs(gamepad1.left_trigger) > 0.33) {
                driveDistanceControl = -100;
                robot.move(driveDistanceControl);
                totalDistance += driveDistanceControl;
            }
            // control: A
            if (gamepad1.a) {
                totalDistance = 0;
            }
            // control: B
            if (gamepad1.b) {
                totalDistance = 0;
            }
            // control: X
            if (gamepad1.x) {
                robot.colorBeacon.blue();
            }
            // control: Y
            if (gamepad1.y) {
                robot.colorBeacon.red();
            }
            if (robot.colorSensor.blue() > 0 && robot.colorSensor.blue() > robot.colorSensor.red()) {
                robot.colorBeacon.blue();
            }
            if (robot.colorSensor.red() > 0 && robot.colorSensor.red() > robot.colorSensor.blue()) {
                robot.colorBeacon.red();
            }
        }
    }
    // ************************** END OP *********************************************************//
}

