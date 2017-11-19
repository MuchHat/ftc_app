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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * {@link Test_Encoders_V9} illustrates how to use the Modern Robotics
 * Range Sensor.
 * <p>
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://modernroboticsinc.com/range-sensor">MR Range Sensor</a>
 */
@TeleOp(name = "Test Encoders V9", group = "Test")
//@Disabled   // comment out or remove this line to enable this opmode
public class Test_Encoders_V9 extends LinearOpMode {

    double target = 0;
    double power = 0.44;

    Team_Hardware_V9 robot = new Team_Hardware_V9();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetryUpdate();

            if (gamepad1.dpad_left) {
                target -= 0.25;
                sleep(100);
            }
            if (gamepad1.dpad_right) {
                target += 0.25;
                sleep(100);
            }
            if (gamepad1.dpad_up) target = 0;
            if (gamepad1.dpad_down) target = 0;

            if (gamepad1.right_bumper) {
                power += 0.11;
                sleep(100);
            }
            if (gamepad1.left_bumper) {
                power -= 0.11;
                sleep(100);
            }

            target = Range.clip(target, 0, 2.0);
            power = Range.clip(power, 0.11, 0.88);

            if (gamepad1.a)
                robot.moveInches(-target, power, 6);
            if (gamepad1.y)
                robot.moveInches(target, power, 6);
            if (gamepad1.b)
                robot.moveSideInches(target, power, 6);
            if (gamepad1.y)
                robot.moveSideInches(-target, power, 6);
       }
    }

    void telemetryUpdate() {
        telemetry.addData("target", "%.2f v", target);
        telemetry.addData("power", "%.2f v", power);
        telemetry.update();
    }
}
