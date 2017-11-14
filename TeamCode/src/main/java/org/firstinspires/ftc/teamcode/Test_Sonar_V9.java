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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link Test_Sonar_V9} illustrates how to use the Modern Robotics
 * Range Sensor.
 * <p>
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://modernroboticsinc.com/range-sensor">MR Range Sensor</a>
 */
@Autonomous(name = "Test Sonar V9", group = "Test")
//@Disabled   // comment out or remove this line to enable this opmode
public class Test_Sonar_V9 extends LinearOpMode {

    ModernRoboticsI2cRangeSensor rangeSensor;

    AnalogSensor frontSonar;
    AnalogSensor leftSonar;
    AnalogSensor rightSonar;

    double leftTarget = 0;
    double rightTarget = 0;
    double frontTarget = 0;

    Team_Hardware_V9 robot = new Team_Hardware_V9();
    double crrError = 0;
    double direction = 0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Front_Range_Sensor");
        frontSonar = hardwareMap.get(AnalogSensor.class, "Front_Sonar");
        leftSonar = hardwareMap.get(AnalogSensor.class, "Left_Sonar");
        rightSonar = hardwareMap.get(AnalogSensor.class, "Right_Sonar");

        waitForStart();

        while (opModeIsActive()) {
            telemetryUpdate();

            if (gamepad1.dpad_left) leftTarget++;
            if (gamepad1.dpad_right) rightTarget++;
            if (gamepad1.dpad_up) frontTarget++;
            if (gamepad1.dpad_down) {
                rightTarget = 0;
                leftTarget = 0;
                frontTarget = 0;
            }
            rightTarget = Range.clip(rightTarget, 0, 1024);
            leftTarget = Range.clip(leftTarget, 0, 1024);
            frontTarget = Range.clip(frontTarget, 0, 1024);

            if (gamepad1.x)
                moveSideBySonar_Test(rightTarget, 0.44, 6, Team_Hardware_V9.SonarPosition.RIGHT);
            if (gamepad1.b)
                moveSideBySonar_Test(leftTarget, 0.44, 6, Team_Hardware_V9.SonarPosition.LEFT);
            if (gamepad1.y)
                moveSideBySonar_Test(frontTarget, 0.44, 6, Team_Hardware_V9.SonarPosition.FRONT);
        }

    }

    void moveSideBySonar_Test(double endPos, double movePower, double timeOutSec, Team_Hardware_V9.SonarPosition sonarPosition) {

        ElapsedTime moveTimer = new ElapsedTime();
        double minPower = 0.22; //TODO
        double rampDown = 22; //TODO
        double crrPower = movePower;
        double crrPos = 0;

        if (sonarPosition == Team_Hardware_V9.SonarPosition.FRONT) {
            crrPos = frontSonar.readRawVoltage();
        } else if (sonarPosition == Team_Hardware_V9.SonarPosition.LEFT) {
            crrPos = leftSonar.readRawVoltage();
        } else {
            crrPos = rightSonar.readRawVoltage();
        }

        crrError = endPos - crrPos;
        direction = crrError > 0 ? 1.0 : -1.0;
        moveTimer.reset();

        while (moveTimer.seconds() < timeOutSec && crrError * direction > 0) {

            robot.leftDistanceControl = 0;
            robot.rightDistanceControl = 0;
            robot.leftDistanceControlBack = 0;
            robot.rightDistanceControlBack = 0;

            crrPower = movePower;
            if (crrError * direction < rampDown) {
                crrPower = minPower;
            }

            if (sonarPosition == Team_Hardware_V9.SonarPosition.FRONT) {
                robot.leftPowerControl = crrPower * direction;
                robot.rightPowerControl = crrPower * direction;
                robot.leftPowerControlBack = crrPower * direction;
                robot.rightPowerControlBack = crrPower * direction;
                robot.setDrivesByPower();

            } else {
                robot.leftPowerControl = -crrPower * direction;
                robot.rightPowerControl = crrPower * direction;
                robot.leftPowerControlBack = crrPower * direction;
                robot.rightPowerControlBack = -crrPower * direction;
                robot.setDrivesByPower();

            }

            robot.waitMillis(111);
            if (sonarPosition == Team_Hardware_V9.SonarPosition.FRONT) {
                crrPos = frontSonar.readRawVoltage();
            } else if (sonarPosition == Team_Hardware_V9.SonarPosition.LEFT) {
                crrPos = leftSonar.readRawVoltage();
            } else {
                crrPos = rightSonar.readRawVoltage();
            }
            crrError = endPos - crrPos;
            telemetryUpdate();
        }
        robot.stopRobot();
    }

    void telemetryUpdate() {
        telemetry.addData("rangeSensor ultrasonic", rangeSensor.rawUltrasonic());
        telemetry.addData("rangeSensor optical", rangeSensor.rawOptical());
        telemetry.addData("rangeSensor cm optical", "%.2f cm", rangeSensor.cmOptical());
        telemetry.addData("rangeSensor cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("frontTarget", "%.2f v", frontTarget);
        telemetry.addData("leftTarget", "%.2f v", leftTarget);
        telemetry.addData("rightTarget", "%.2f v", rightTarget);
        telemetry.addData("frontSonar", "%.2f v", (double) frontSonar.readRawVoltage());
        telemetry.addData("leftSonar", "%.2f v", (double) leftSonar.readRawVoltage());
        telemetry.addData("rightSonar", "%.2f v", (double) rightSonar.readRawVoltage());
        telemetry.addData("crrError", "%.2f v", crrError);
        telemetry.addData("direction", "%.2f", direction);

        telemetry.update();
    }
}
