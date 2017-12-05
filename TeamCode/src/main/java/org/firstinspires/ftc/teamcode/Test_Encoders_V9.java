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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

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

    double target = 11;
    double power = 0.66;
    double crrPower = 0;
    double crrDrift = 0;
    double crrReduceRatio = 1;
    double crrLockedLeft = 0;
    double crrLockedRight = 0;

    Team_Hardware_V9 robot = new Team_Hardware_V9();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        target = 11;
        power = 0.66;
        crrPower = 0;
        crrDrift = 0;
        crrReduceRatio = 1;
        crrLockedLeft = 0;
        crrLockedRight = 0;

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
            if (gamepad1.dpad_down) power = 0;

            if (gamepad1.right_bumper) {
                power += 0.11;
                sleep(100);
            }
            if (gamepad1.left_bumper) {
                power -= 0.11;
                sleep(100);
            }

            target = Range.clip(target, 0, 40.0);
            power = Range.clip(power, 0, 1);

            robot.moveLinearGyroTrackingEnabled = true;
            robot.moveLinearGyroHeadingToTrack = 0;

            if (gamepad1.a)
                moveInches(-target, power, 11);
            if (gamepad1.y)
                moveInches(target, power, 11);
            if (gamepad1.x)
                moveInchesStopOnFlat(-target, power, 11);
            if (gamepad1.b)
                moveInchesStopOnFlat(target, power, 11);
        }
    }

    double pulsesToMm(double distancePulses) {

        //convert from pulses to mm
        double correction = 4.75;
        double revolutions = distancePulses / ((7 * 60) * correction);

        double distanceMM = revolutions * ((25.4 * 4) * Math.PI);

        return distanceMM;
    }

    double mmToPulses(double distanceMM) {

        //convert from mm to pulses
        double correction = 4.75;
        double revolutions = distanceMM / ((25.4 * 4) * Math.PI); //a 4 inch wheel
        double distancePulses = revolutions * (7 * 60) * correction; // 420 tics per revolution

        return distancePulses;
    }

    void moveLinear(double distanceMM, double power, double timeOut, double dirFrontLeft, double dirFrontRight, double dirBackLeft, double dirBackRight) {

        double distanceLowSpeedMM = 99;

        double lowSpeedPower = 0.66;
        double highSpeedPower = 1.0;

        if (power != 0) {
            lowSpeedPower = Math.abs(power);
            highSpeedPower = Math.abs(power);
        }

        double distancePulses = mmToPulses(distanceMM);
        double crrPower = Math.abs(distanceMM) > distanceLowSpeedMM ? highSpeedPower : lowSpeedPower;

        robot.leftPowerControl = crrPower;
        robot.rightPowerControl = crrPower;
        robot.leftPowerControlBack = crrPower;
        robot.rightPowerControlBack = crrPower;

        robot.leftDistanceControl = distancePulses * dirFrontLeft;
        robot.rightDistanceControl = distancePulses * dirFrontRight;
        robot.leftDistanceControlBack = distancePulses * dirBackLeft;
        robot.rightDistanceControlBack = distancePulses * dirBackRight;

        setDrivesByDistance(timeOut);
        stopRobot();
    }

    boolean isFlat() {

        boolean xFlat = (robot.imuGyro.getInclineX() <= 5) || (robot.imuGyro.getInclineX() >= 355); //CHANGE
        boolean yFlat = (robot.imuGyro.getInclineY() <= 3) || (robot.imuGyro.getInclineY() >= 353);

        return xFlat && yFlat;
    }

    double inchesToTarget() {

        double leftToTarget = Math.abs(robot.targetLeft - robot.leftDrive.getCurrentPosition());
        double rightToTarget = Math.abs(robot.targetRight - robot.rightDrive.getCurrentPosition());
        double leftBackToTarget = Math.abs(robot.targetLeftBack - robot.leftDriveBack.getCurrentPosition());
        double rightBackToTarget = Math.abs(robot.targetRightBack - robot.rightDriveBack.getCurrentPosition());

        return pulsesToMm((leftToTarget + rightToTarget + leftBackToTarget + rightBackToTarget) / 4) / 24.5; //TODO

    }

    double gyroDrift(double targetHeading) {

        double crrHeading = robot.imuGyro.getHeading();

        if (targetHeading >= 180 && crrHeading < 180) {
            crrHeading += 360;
        }
        if (targetHeading <= 180 && crrHeading > 180) {
            crrHeading -= 360;
        }

        // if small drift ignore
        if (Math.abs(targetHeading - crrHeading) < 7) {
            return 0;
        }

        // 1.0 means 90 deg off
        // > 0 means drift to right
        // < 0 means drift to left
        double drift = (targetHeading - crrHeading) / 90; // 0.07 is min

        return drift;
    }

    void moveInches(double distanceInches, double movePower, double timeOut) {

        robot.moveLinearStopOnFlatEnabled = false;
        moveLinear(distanceInches * 24.5, movePower, timeOut, 1.0, 1.0, 1.0, 1.0);
    }

    void moveInchesStopOnFlat(double distanceInches, double movePower, double timeOut) {

        robot.moveLinearStopOnFlatEnabled = true;
        moveLinear(distanceInches * 24.5, movePower, timeOut, 1.0, 1.0, 1.0, 1.0);
    }

    void setDrivesByDistance(double timeOut) {

        ElapsedTime encodersTimer = new ElapsedTime();
        int prevBeaconColor = robot.colorBeacon.getColorNumber();
        double lastCheckIfLocked = 0;

        if (robot.moveLinearGyroTrackingEnabled) {
            if (robot.moveLinearGyroHeadingToTrack < 0) {
                robot.moveLinearGyroHeadingToTrack = robot.imuGyro.getHeading();
            }
        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftDriveBack.setPower(0);
        robot.rightDriveBack.setPower(0);

        robot.leftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        robot.rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        robot.targetLeft = robot.leftDrive.getCurrentPosition() + (int) robot.leftDistanceControl;
        robot.targetRight = robot.rightDrive.getCurrentPosition() + (int) robot.rightDistanceControl;
        robot.targetLeftBack = robot.leftDriveBack.getCurrentPosition() + (int) robot.leftDistanceControlBack;
        robot.targetRightBack = robot.rightDriveBack.getCurrentPosition() + (int) robot.rightDistanceControlBack;

        robot.leftDrive.setTargetPosition(robot.targetLeft);
        robot.rightDrive.setTargetPosition(robot.targetRight);
        robot.leftDriveBack.setTargetPosition(robot.targetLeftBack);
        robot.rightDriveBack.setTargetPosition(robot.targetRightBack);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftPowerControl = Range.clip(robot.leftPowerControl, 0, 1);
        robot.rightPowerControl = Range.clip(robot.rightPowerControl, 0, 1);
        robot.leftPowerControlBack = Range.clip(robot.leftPowerControlBack, 0, 1);
        robot.rightPowerControlBack = Range.clip(robot.rightPowerControlBack, 0, 1);

        robot.leftDrive.setPower(Math.abs(robot.leftPowerControl));
        robot.rightDrive.setPower(Math.abs(robot.rightPowerControl));
        robot.leftDriveBack.setPower(Math.abs(robot.leftPowerControlBack));
        robot.rightDriveBack.setPower(Math.abs(robot.rightPowerControlBack));

        double timeOutSec = 4;
        if (timeOut > 0) {
            timeOutSec = timeOut;
        }
        encodersTimer.reset();

        boolean locked = false;
        double leftPrev = robot.leftDrive.getCurrentPosition();
        double rightPrev = robot.rightDrive.getCurrentPosition();
        double leftBackPrev = robot.leftDriveBack.getCurrentPosition();
        double rightBackPrev = robot.rightDriveBack.getCurrentPosition();

        // LOOP UNTIL TARGET HIT OR TIMEOUT
        while (encodersTimer.seconds() < timeOutSec) {

            // WAIT LOOP AND CHECK IF MOTORS STOPPED
            boolean stillRunning = true;
            for (int i = 0; i < 6; i++) {
                waitMillis(11);
                telemetryUpdate();
                stillRunning = robot.leftDrive.isBusy() ||
                        robot.rightDrive.isBusy() ||
                        robot.leftDriveBack.isBusy() ||
                        robot.rightDriveBack.isBusy();
                if (!stillRunning) {
                    break;
                }
                stillRunning = inchesToTarget() > 0.05;
                if (!stillRunning) {
                    break;
                }
            }
            if (!stillRunning) {
                break;
            }
            telemetryUpdate();

            // END WAIT LOOP AND CHECK IF MOTORS STOPPED
            // SLOW DOWN FOR STOP ON FLAT
            if (robot.moveLinearStopOnFlatEnabled &&
                    (robot.leftDrive.getPower() > 0.11 ||
                            robot.leftDrive.getPower() > 0.11 ||
                            robot.leftDriveBack.getPower() > 0.11 ||
                            robot.rightDriveBack.getPower() > 0.11) &&
                    inchesToTarget() < robot.moveLinearStopOnFlatRampDownInches) {

                double rampDownRatio = 0.88; // reduces by half in 1/2 sec

                robot.leftDrive.setPower(robot.leftDrive.getPower() * rampDownRatio);
                robot.rightDrive.setPower(robot.rightDrive.getPower() * rampDownRatio);
                robot.leftDriveBack.setPower(robot.leftDriveBack.getPower() * rampDownRatio);
                robot.rightDriveBack.setPower(robot.rightDriveBack.getPower() * rampDownRatio);

                // WHITE FLASH MEANS SLOW DOWN FOR RAMP DOWN
                robot.colorBeacon.white();
            }
            // END SLOW DOWN
            // STOP ON FLAT
            if (robot.moveLinearStopOnFlatEnabled &&
                    isFlat() &&
                    inchesToTarget() < robot.moveLinearStopOnFlatRampDownInches) {

                // WHITE BLINK MEANS STOPPED ON FLAT
                robot.colorBeacon.purple();

                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.leftDriveBack.setPower(0);
                robot.rightDriveBack.setPower(0);

                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                stillRunning = false;
                telemetryUpdate();
                break;
            }
            // END STOP ON FLAT
            // GYRO TRACKING IF ENABLED
            double driftRight = 0;
            crrReduceRatio = 1;
            if (robot.moveLinearGyroTrackingEnabled) {
                driftRight = gyroDrift(robot.moveLinearGyroHeadingToTrack);
                crrDrift = driftRight;
            }
            if (robot.moveLinearGyroTrackingEnabled && driftRight != 0 && stillRunning) {

                // PINK MEANS GYRO CORRECTING
                robot.colorBeacon.pink();

                double lPower = (robot.leftDrive.getPower() + robot.leftDriveBack.getPower()) / 2;
                double rPower = (robot.rightDriveBack.getPower() + robot.rightDriveBack.getPower()) / 2;

                double direction = robot.rightDistanceControl > 0 ? 1.0 : -1.0;

                // reduce power on one side to compensate
                // set to 0.88 for 11 deg drift
                // set to 0.11 for 90 deg drift
                double reduceRatio = 1 - 8 * Math.abs(driftRight); //CHANGE
                reduceRatio = Range.clip(reduceRatio, 0.44, 0.77);
                crrReduceRatio = reduceRatio;

                if (driftRight * direction > 0) {

                    lPower = rPower * reduceRatio; // x0.07 is the minimum for 6 deg drift

                } else {

                    rPower = lPower * reduceRatio;

                }

                lPower = Range.clip(lPower, 0.08, 1);
                rPower = Range.clip(rPower, 0.08, 1);

                robot.leftDrive.setPower(Math.abs(lPower));
                robot.rightDrive.setPower(Math.abs(rPower));
                robot.leftDriveBack.setPower(Math.abs(lPower));
                robot.rightDriveBack.setPower(Math.abs(rPower));

            } else if (robot.moveLinearGyroTrackingEnabled && driftRight == 0 && stillRunning) {

                // DONE ADJUSTING
                robot.colorBeacon.colorNumber(prevBeaconColor);

                double maxPower = Math.max(
                        Math.max(robot.leftDrive.getPower(),robot.leftDriveBack.getPower()),
                        Math.max(robot.rightDriveBack.getPower(),robot.rightDriveBack.getPower()));
                double minPower = Math.min(
                        Math.min(robot.leftDrive.getPower(),robot.leftDriveBack.getPower()),
                        Math.min(robot.rightDriveBack.getPower(),robot.rightDriveBack.getPower()));

                maxPower = Range.clip(maxPower, 0, 1);
                minPower = Range.clip(minPower, 0, 1);

                if (maxPower - minPower > 0.03 || maxPower < 0.08 ) {

                    maxPower = Range.clip(maxPower, 0.08, 1);

                    robot.leftDrive.setPower(Math.abs(maxPower));
                    robot.rightDrive.setPower(Math.abs(maxPower));
                    robot.leftDriveBack.setPower(Math.abs(maxPower));
                    robot.rightDriveBack.setPower(Math.abs(maxPower));
                }
            }
            if (robot.moveLinearGyroTrackingEnabled && stillRunning) {

                // ADJUST TARGETS TOO
                double leftPulsesToDestination =
                        (Math.abs(robot.leftDrive.getTargetPosition() - robot.leftDrive.getCurrentPosition()) +
                                Math.abs(robot.leftDriveBack.getTargetPosition() - robot.leftDriveBack.getCurrentPosition())) / 2;
                double rightPulsesToDestination =
                        (Math.abs(robot.rightDrive.getTargetPosition() - robot.rightDrive.getCurrentPosition()) +
                                Math.abs(robot.rightDriveBack.getTargetPosition() - robot.rightDriveBack.getCurrentPosition())) / 2;

                if (Math.abs(leftPulsesToDestination - rightPulsesToDestination) > 11) {
                    if (leftPulsesToDestination > rightPulsesToDestination) {

                        double newLeftTarget = robot.leftDrive.getCurrentPosition() +
                                rightPulsesToDestination / leftPulsesToDestination * (robot.leftDrive.getTargetPosition() - robot.leftDrive.getCurrentPosition());
                        robot.leftDrive.setTargetPosition((int) newLeftTarget);

                        double newLeftBackTarget = robot.leftDriveBack.getCurrentPosition() +
                                rightPulsesToDestination / leftPulsesToDestination * (robot.leftDriveBack.getTargetPosition() - robot.leftDriveBack.getCurrentPosition());
                        robot.leftDriveBack.setTargetPosition((int) newLeftBackTarget);

                    } else {
                        double newRightTarget = robot.rightDrive.getCurrentPosition() +
                                leftPulsesToDestination / rightPulsesToDestination * (robot.rightDrive.getTargetPosition() - robot.rightDrive.getCurrentPosition());
                        robot.rightDrive.setTargetPosition((int) newRightTarget);

                        double newRightBackTarget = robot.rightDriveBack.getCurrentPosition() +
                                leftPulsesToDestination / rightPulsesToDestination * (robot.rightDriveBack.getTargetPosition() - robot.rightDriveBack.getCurrentPosition());
                        robot.rightDriveBack.setTargetPosition((int) newRightBackTarget);
                    }
                }
            }
            // END GYRO TRACKING
            // CHECK IF WHEELS LOCKED
            if ((encodersTimer.milliseconds() - lastCheckIfLocked > 222) && stillRunning) {

                lastCheckIfLocked = encodersTimer.milliseconds();

                double lMove = Math.abs(robot.leftDrive.getCurrentPosition() - leftPrev);
                double rMove = Math.abs(robot.rightDrive.getCurrentPosition() - rightPrev);
                double lbMove = Math.abs(robot.leftDriveBack.getCurrentPosition() - leftBackPrev);
                double rbMove = Math.abs(robot.rightDriveBack.getCurrentPosition() - rightBackPrev);

                double lDiffPercent = Math.abs(lMove - lbMove) / Math.max(Math.max(lMove, lbMove), 1);
                double rDiffPercent = Math.abs(rMove - rbMove) / Math.max(Math.max(rMove, rbMove), 1);
                double maxMove = Math.max(Math.max(Math.max(lMove, rMove), lbMove), rbMove);

                leftPrev = robot.leftDrive.getCurrentPosition();
                rightPrev = robot.rightDrive.getCurrentPosition();
                leftBackPrev = robot.leftDriveBack.getCurrentPosition();
                rightBackPrev = robot.rightDriveBack.getCurrentPosition();

                // diff is bigger than 33%
                // do not check for lock if moving slowly at the end
                locked = (Math.abs(lMove - lbMove) > 444 || Math.abs(rMove - rbMove) > 444); // TODO
                crrLockedLeft = Math.abs(lMove - lbMove);
                crrLockedRight = Math.abs(rMove - rbMove);
                telemetryUpdate();

                //if one wheel is locked stop and restart,
                //check for stall 222ms after start not sooner
                if (locked && encodersTimer.seconds() > 0.22) {

                    robot.leftDrive.setPower(0);
                    robot.rightDrive.setPower(0);
                    robot.leftDriveBack.setPower(0);
                    robot.rightDriveBack.setPower(0);

                    // YELLOW MEANS LOCKED UP
                    robot.colorBeacon.yellow();
                    beaconBlink(2); // takes care of wait too

                    robot.leftDrive.setPower(Math.abs(robot.leftPowerControl));
                    robot.rightDrive.setPower(Math.abs(robot.rightPowerControl));
                    robot.leftDriveBack.setPower(Math.abs(robot.leftPowerControlBack));
                    robot.rightDriveBack.setPower(Math.abs(robot.rightPowerControlBack));
                }
                if (!locked) {
                    robot.colorBeacon.colorNumber(prevBeaconColor);
                }
            }
            //END CHECK IF LOCKED
        }
        // END LOOP UNTIL TARGET HIT OR TIMEOUT

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftDriveBack.setPower(0);
        robot.rightDriveBack.setPower(0);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (robot.moveLinearGyroTrackingEnabled) {
            robot.moveLinearGyroTrackingEnabled = false;
            robot.moveLinearGyroHeadingToTrack = -1.0;
            robot.colorBeacon.colorNumber(prevBeaconColor);
        }
        if (robot.moveLinearStopOnFlatEnabled) {
            robot.moveLinearStopOnFlatEnabled = false;
            robot.colorBeacon.colorNumber(prevBeaconColor);
        }
    }

    void setDrivesByPower() {

        robot.leftPowerControl = Range.clip(robot.leftPowerControl, -1, 1);
        robot.rightPowerControl = Range.clip(robot.rightPowerControl, -1, 1);
        robot.leftPowerControlBack = Range.clip(robot.leftPowerControlBack, -1, 1);
        robot.rightPowerControlBack = Range.clip(robot.rightPowerControlBack, -1, 1);

        robot.liftControl = Range.clip(robot.liftControl, -1, 1);

        if (robot.liftControl >= 0 && robot.topSwitch.getState()) { // true means switch is pressed
            robot.liftControl = 0;
        }
        if (robot.liftControl < 0 && robot.bottomSwitch.getState()) { // true means switch is pressed
            robot.liftControl = 0;
        }
        if (robot.liftControl < 0) {
            robot.liftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.liftDrive.setPower(Math.abs(robot.liftControl));
        }
        if (robot.liftControl >= 0) {
            robot.liftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.liftDrive.setPower(Math.abs(robot.liftControl));
        }

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (robot.leftPowerControl >= 0) {
            robot.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftDrive.setPower(Math.abs(robot.leftPowerControl));
        }
        if (robot.leftPowerControl < 0) {
            robot.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.leftDrive.setPower(Math.abs(robot.leftPowerControl));
        }
        if (robot.rightPowerControl >= 0) {
            robot.rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.rightDrive.setPower(Math.abs(robot.rightPowerControl));
        }
        if (robot.rightPowerControl < 0) {
            robot.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive.setPower(Math.abs(robot.rightPowerControl));
        }
        if (robot.leftPowerControlBack >= 0) {
            robot.leftDriveBack.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftDriveBack.setPower(Math.abs(robot.leftPowerControlBack));
        }
        if (robot.leftPowerControlBack < 0) {
            robot.leftDriveBack.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.leftDriveBack.setPower(Math.abs(robot.leftPowerControlBack));
        }
        if (robot.rightPowerControlBack >= 0) {
            robot.rightDriveBack.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.rightDriveBack.setPower(Math.abs(robot.rightPowerControlBack));
        }
        if (robot.rightPowerControlBack < 0) {
            robot.rightDriveBack.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDriveBack.setPower(Math.abs(robot.rightPowerControlBack));
        }
    }

    void stopRobot() {
        robot.leftPowerControl = 0;
        robot.rightPowerControl = 0;
        robot.leftPowerControlBack = 0;
        robot.rightPowerControlBack = 0;

        robot.leftPowerControl = 0;
        robot.rightPowerControl = 0;
        robot.leftPowerControlBack = 0;
        robot.rightPowerControlBack = 0;

        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);
        robot.rightDriveBack.setPower(0);
        robot.leftDriveBack.setPower(0);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void beaconBlink(int count) {
        int color = robot.colorBeacon.getColorNumber();

        for (int i = 0; i < count; i++) {
            robot.colorBeacon.off();
            waitMillis(33);
            robot.colorBeacon.colorNumber(color);
        }
    }

    void waitMillis(double millis) {
        sleep((long) millis);
    }

    public enum SonarPosition {
        FRONT,
        LEFT,
        RIGHT
    }

    void telemetryUpdate() {
        telemetry.addData("target", "%.2f inches", target);
        telemetry.addData("power", "%.2f power", power);
        telemetry.addData("inchesToTarget", "%.2f inches", robot.inchesToTarget());
        telemetry.addData("crrPower", "%.2f power", crrPower);
        telemetry.addData("crrDrift", "%.2f drift", crrDrift);
        telemetry.addData("is flat", "%.2f flat", isFlat() ? 1.0 : 0.0);
        telemetry.addData("reduce ratio", "%.2f ratio", crrReduceRatio);
        telemetry.addData("locked left", "%.2f pulses", crrLockedLeft);
        telemetry.addData("locked right", "%.2f pulses", crrLockedRight);
        telemetry.addData("locked diff", "%.2f pulses", (double) (crrLockedRight - crrLockedLeft));
        telemetry.addData("incline X", "%.2f", (double) robot.imuGyro.getInclineX());
        telemetry.addData("incline Y", "%.2f", (double) robot.imuGyro.getInclineY());
        telemetry.update();
    }
}
