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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

// ************************** ROBOT HW CLASS *****************************************************//

public class Team_Hardware_V9 {

    // ************************** ROBOT HW VARIABLES *********************************************//

    public DigitalChannel topSwitch = null;
    public DigitalChannel bottomSwitch = null;
    public ColorSensor colorSensor = null;
    public HardwareMap hwMap = null;
    public ElapsedTime runtime = new ElapsedTime();
    public ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    public IntegratingGyroscope gyro;
    public MRIColorBeacon colorBeacon;
    //********************************* MOVE STATES **********************************************//
    double leftPowerControl = 0;
    double rightPowerControl = 0;
    double leftPowerControlBack = 0;
    double rightPowerControlBack = 0;

    double leftDistanceControl = 0;
    double rightDistanceControl = 0;
    double leftDistanceControlBack = 0;
    double rightDistanceControlBack = 0;

    double headingControl = 0;
    double liftControl = 0;
    double leftClawControl = 0;
    double rightClawControl = 0;
    double baseControl = 0;
    double elbowControl = 0;
    double gameStartHeading = 0;
    //********************************* PREDEFINED POS *******************************************//
    double clawClosed[] = {0.95, 0.10};
    double clawZero[] = {0.22, 0.75};
    double clawOpen[] = {0.60, 0.40};
    double driveDefaultSpeed = 0.33;
    double armPosZero[] = {0, 0};
    // ************************** MAIN LOOP ******************************************************//
    double turnDefaultSpeed = 0.16;
    double servoDefaultSpeed = 0.00033;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor rightDriveBack = null;
    private DcMotor leftDriveBack = null;
    private DcMotor liftDrive = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo base = null;
    private Servo elbow = null;

    // ************************** HW CONSTRUCTOR  ************************************************//

    public Team_Hardware_V9() {

    }

    // ************************** HW INIT  *******************************************************//

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        leftDrive = hwMap.get(DcMotor.class, "Motor_Left");
        rightDrive = hwMap.get(DcMotor.class, "Motor_Right");
        leftDriveBack = hwMap.get(DcMotor.class, "Motor_Left_Back");
        rightDriveBack = hwMap.get(DcMotor.class, "Motor_Right_Back");
        liftDrive = hwMap.get(DcMotor.class, "Motor_Lift");

        topSwitch = hwMap.get(DigitalChannel.class, "Switch_Top");
        bottomSwitch = hwMap.get(DigitalChannel.class, "Switch_Bottom");

        base = hwMap.get(Servo.class, "Base");
        elbow = hwMap.get(Servo.class, "Elbow");

        leftClaw = hwMap.get(Servo.class, "Claw_Left");
        rightClaw = hwMap.get(Servo.class, "Claw_Right");

        colorSensor = hwMap.get(ColorSensor.class, "Color_Sensor");

        modernRoboticsI2cGyro = hwMap.get(ModernRoboticsI2cGyro.class, "Gyro");
        gyro = modernRoboticsI2cGyro;

        colorBeacon = new MRIColorBeacon();
        colorBeacon.init(hwMap, "Beacon");

        colorSensor.enableLed(false);

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        liftDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        liftDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftPowerControl = 0;
        rightPowerControl = 0;
        leftPowerControlBack = 0;
        rightPowerControlBack = 0;
        leftClawControl = clawZero[0];
        rightClawControl = clawZero[1];
        baseControl = armPosZero[0];
        elbowControl = armPosZero[1];
        setDrives();
        setServos();
    }

    // ************************** MANUAL DRIVE HELPER FUNCTIONS  *********************************//

    void turnToHeading(double newHeading) {

        newHeading %= 360;

        double crrHeading = modernRoboticsI2cGyro.getHeading();
        double diffAbs = Math.abs(newHeading - crrHeading);
        double diff360Abs = 360 - diffAbs;
        double direction = newHeading > crrHeading ? 1.0 : -1.0;
        double inverted = diffAbs > diff360Abs ? 1.0 : -1.0;

        double turnDeg = inverted > 0 ? diffAbs : diff360Abs;
        turnDeg *= direction * inverted;

        turn(turnDeg);
    }

    void turn(double turnDeg) {

        turnDeg *= -1;

        double accelDistance = 6; // accelerate to max over 11 deg
        double brakeDistance = 44; // ramp down the last 15 deg

        double minPower = 0.11;
        double maxPower = 0.29;

        double startHeading = modernRoboticsI2cGyro.getHeading();
        double endHeading = startHeading + turnDeg;

        double direction = turnDeg > 0 ? 1.0 : -1.0;
        double crrError = turnDeg * direction;

        double iterations = 0;

        while (crrError > 0 && iterations < 3333) {

            double crrPower = maxPower;
            double crrDistance = turnDeg * direction - crrError;

            if (crrDistance < accelDistance) {
                crrPower = minPower + getS(crrDistance / accelDistance) * (maxPower - minPower);
            }
            if (crrError < brakeDistance) {
                crrPower = minPower + getS(crrError / brakeDistance) * (maxPower - minPower);
            }

            crrPower = Range.clip(crrPower, minPower, maxPower);

            leftPowerControl = -crrPower * direction;
            rightPowerControl = crrPower * direction;
            leftPowerControlBack = leftPowerControl;
            rightPowerControlBack = rightPowerControl;

            setDrives();

            double stepMillis = 1;
            waitMillis(stepMillis);
            if (Math.abs(crrError) < 30) {
                stopRobot();
                waitMillis(2 * stepMillis);
            }

            double crrHeading = modernRoboticsI2cGyro.getHeading();

            if (startHeading >= 180 && direction > 0 && crrHeading < 180) {
                crrHeading += 360;
            }
            if (startHeading <= 180 && direction < 0 && crrHeading > 180) {
                crrHeading -= 360;
            }
            crrError = (endHeading - crrHeading) * direction;

            iterations++;
        }
        stopRobot();
        headingControl = modernRoboticsI2cGyro.getHeading();
    }

    void openClaw() {
        leftClawControl = clawOpen[0];
        rightClawControl = clawOpen[1];
        setServos();
    }

    void closeClaw() {
        leftClawControl = clawClosed[0];
        rightClawControl = clawClosed[1];
        setServos();
    }

    void moveLift(double distance) {
        double mmMillis = 0.3;
        double stepTime = Math.abs(distance) / mmMillis;
        double liftDefaultPower = 0.88;

        liftControl = liftDefaultPower;
        setDrives();

        waitMillis(stepTime);

        liftControl = 0;
        setDrives();
    }

    void move(double distance) {

        double movePower = 0.66;

        moveLinear(distance, movePower, 1.0, 1.0, 1.0, 1.0);
    }

    void moveSide(double distance) {

        double sideMovePower = 0.44;

        moveLinear(distance, sideMovePower, 1.0, -1.0, -1.0, 1.0);
    }

    void moveLinear(double distance, double power, double dirFrontLeft, double dirFrontRight, double dirBackLeft, double dirBackRight) {

        double brakeDistance = 22; //ramp down the last 22 mm

        double minPower = 0.11;
        double maxPower = 0.99;

        power = Range.clip(power, minPower, maxPower);

        double direction = distance > 0 ? 1.0 : -1.0;

        double firstSegmentPower = power;
        double secondSegmentPower = minPower;

        double firstSegment = distance;
        double secondSegment = 0;

        while (direction * distance > brakeDistance && power > minPower) {

            firstSegment = distance - brakeDistance * direction;
            secondSegment = brakeDistance * direction;
        }

        leftPowerControl = firstSegmentPower;
        rightPowerControl = firstSegmentPower;
        leftPowerControlBack = firstSegmentPower;
        rightPowerControlBack = firstSegmentPower;

        leftDistanceControl = firstSegment;
        rightDistanceControl = firstSegment;
        leftDistanceControlBack = firstSegment;
        rightPowerControlBack = firstSegmentPower;

        setDrives();

        if (secondSegment != 0) {

            leftPowerControl = secondSegmentPower;
            rightPowerControl = secondSegmentPower;
            leftPowerControlBack = secondSegmentPower;
            rightPowerControlBack = secondSegmentPower;

            leftDistanceControl = secondSegment;
            rightDistanceControl = secondSegment;
            leftDistanceControlBack = secondSegment;
            rightPowerControlBack = secondSegment;

            setDrives();

            leftPowerControl = 0;
            rightPowerControl = 0;
            leftPowerControlBack = 0;
            rightPowerControlBack = 0;
        }
        stopRobot();
    }

    double getS(double ratio) {

        double jerk = 1.355;
        double div = ((1 - Math.cos(Math.pow(0.5, jerk) * Math.PI)) / 2) / 2;
        ratio = Range.clip(ratio, 0, 1);
        double s = ratio;

        if (ratio <= 0.5) {
            s = ((1 - Math.cos(Math.pow(ratio, jerk) * Math.PI)) / 2) / div / 2;
            s = Range.clip(s, 0, 0.5);
        } else if (ratio > 0.5) {
            s = 1 - (((1 - Math.cos(Math.pow(1 - ratio, jerk) * Math.PI)) / 2) / div / 2);
            s = Range.clip(s, 0.5, 1);
        }

        return Range.clip(s, 0.01, 1);
    }

    // ************************** ARM  DRIVE SERVOS HELPER FUNCTIONS  ****************************//

    void moveArmPosZero() {
        moveArm(armPosZero[0], armPosZero[1]);
    }

    void moveArm(double newBase, double newElbow) {

        double stepSize = 0.004;
        double stepsAccel = 11;
        double stepsBrake = 22;
        double defaultTime = 2;
        double minStepTime = 2;
        double maxStepTime = 6;

        double baseStart = baseControl;
        double elbowStart = elbowControl;

        double stepCount = Math.abs(baseStart - newBase) / stepSize;
        stepCount = Math.max(Math.abs(elbowStart - newElbow) / stepSize, stepCount);
        stepCount = Range.clip(stepCount, 11, 333); //should not be more than 999 steps

        double elbowStepSize = (newElbow - baseStart) / stepCount;
        double baseStepSize = (newBase - elbowStart) / stepCount;

        for (int i = 0; i < (int) stepCount; i++) {

            double baseCrr = baseStart + i * baseStepSize;
            double elbowCrr = elbowStart + i * elbowStepSize;

            baseControl = baseCrr;
            elbowControl = elbowCrr;

            baseControl = Range.clip(baseControl, Math.min(baseStart, newBase), Math.max(baseStart, newBase));
            elbowControl = Range.clip(elbowControl, Math.min(elbowStart, newElbow), Math.max(elbowStart, newElbow));
            setServos();

            double crrStepTime = defaultTime;
            if (i < stepsAccel) {
                double ratio = getS((stepsAccel - i) / stepsAccel);
                crrStepTime = defaultTime * 1 / ratio;
            }
            if (stepCount - i < stepsBrake) {
                double ratio = getS((stepCount - i - stepsBrake) / stepsBrake);
                crrStepTime = defaultTime * 1 / ratio;
            }
            crrStepTime = Range.clip(crrStepTime, minStepTime, maxStepTime);
            waitMillis(crrStepTime);
        }

        baseControl = newBase;
        elbowControl = newElbow;
        setServos();
    }

    // ************************** HARDWARE SET FUNCTIONS *****************************************//

    void setDrives() {

        if (leftDistanceControl == 0 &&
                leftDistanceControlBack == 0 &&
                rightDistanceControl == 0 &&
                rightDistanceControlBack == 0) {
            setDrivesByPower();
        } else {
            setDrivesByDistance();
        }
    }

    void setDrivesByDistance() {

        if (leftPowerControl == 0 &&
                rightPowerControl == 0 &&
                leftPowerControlBack == 0 &&
                rightPowerControlBack == 0) {
            stopRobot();
            return;
        }
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) leftDistanceControl);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) rightDistanceControl);
        leftDriveBack.setTargetPosition(leftDriveBack.getCurrentPosition() + (int) leftDistanceControlBack);
        rightDriveBack.setTargetPosition(rightDriveBack.getCurrentPosition() + (int) rightDistanceControlBack);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();

        double timeOutSec = 6;

        leftDrive.setPower(Math.abs(leftPowerControl));
        rightDrive.setPower(Math.abs(rightPowerControl));
        leftDriveBack.setPower(Math.abs(leftPowerControlBack));
        rightDriveBack.setPower(Math.abs(rightPowerControlBack));

        while ((runtime.seconds() < timeOutSec) &&
                (leftDrive.isBusy() && rightDrive.isBusy() && leftDrive.isBusy() && leftDrive.isBusy())) {
            waitMillis(1);
        }

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopRobot();
    }

    void setDrivesByPower() {

        leftPowerControl = Range.clip(leftPowerControl, -1, 1);
        rightPowerControl = Range.clip(rightPowerControl, -1, 1);
        leftPowerControlBack = Range.clip(leftPowerControlBack, -1, 1);
        rightPowerControlBack = Range.clip(rightPowerControlBack, -1, 1);

        liftControl = Range.clip(liftControl, -1, 1);

        if (liftControl >= 0 && topSwitch.getState()) { // true means switch is pressed
            liftControl = 0;
        }
        if (liftControl < 0 && bottomSwitch.getState()) { // true means switch is pressed
            liftControl = 0;
        }
        if (liftControl < 0) {
            liftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            liftDrive.setPower(Math.abs(liftControl));
        }
        if (liftControl >= 0) {
            liftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            liftDrive.setPower(Math.abs(liftControl));
        }

        // if gyro indicates drifting add same power to correct
        // do not add if already doing a turn
        double headingCorrection = 0;

        if (leftPowerControl == rightPowerControl &&
                leftPowerControl == rightPowerControlBack &&
                leftPowerControl == leftPowerControlBack) {
            double headingCrr = modernRoboticsI2cGyro.getHeading();
            double error;
            double driveDirection = leftPowerControl > 0 ? 1.0 : -1.0;

            if (headingControl > 180 && headingCrr <= 180 && headingControl - headingCrr > 180) {
                headingCrr += 360;
            }
            if (headingControl <= 180 && headingCrr > 180 && headingCrr - headingControl > 180) {
                headingCrr -= 360;
            }
            error = headingCrr - headingControl;
            error = Range.clip(error, -45, 45); // if completely off trim down

            headingCorrection = error / 45 * 0.11; //TODO tune up the amount of correction
            headingCorrection *= driveDirection; // apply correction the other way when running in reverse

            if (leftPowerControl == 0) headingCorrection = 0;
        }

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (leftPowerControl >= 0) {
            leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            leftDrive.setPower(Math.abs(leftPowerControl) - headingCorrection);
        }
        if (leftPowerControl < 0) {
            leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            leftDrive.setPower(Math.abs(leftPowerControl) - headingCorrection);
        }
        if (rightPowerControl >= 0) {
            rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightDrive.setPower(Math.abs(rightPowerControl) + headingCorrection);
        }
        if (rightPowerControl < 0) {
            rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rightDrive.setPower(Math.abs(rightPowerControl) + headingCorrection);
        }
        if (leftPowerControlBack >= 0) {
            leftDriveBack.setDirection(DcMotorSimple.Direction.REVERSE);
            leftDriveBack.setPower(Math.abs(leftPowerControlBack) - headingCorrection);
        }
        if (leftPowerControlBack < 0) {
            leftDriveBack.setDirection(DcMotorSimple.Direction.FORWARD);
            leftDriveBack.setPower(Math.abs(leftPowerControlBack) - headingCorrection);
        }
        if (rightPowerControlBack >= 0) {
            rightDriveBack.setDirection(DcMotorSimple.Direction.FORWARD);
            rightDriveBack.setPower(Math.abs(rightPowerControlBack) + headingCorrection);
        }
        if (rightPowerControlBack < 0) {
            rightDriveBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightDriveBack.setPower(Math.abs(rightPowerControlBack) + headingCorrection);
        }
    }

    void setServos() {

        double minLeftClaw = Math.min(Math.min(clawZero[0], clawClosed[0]), clawOpen[0]);
        double maxLeftClaw = Math.max(Math.max(clawZero[0], clawClosed[0]), clawOpen[0]);

        double minRightClaw = Math.min(Math.min(clawZero[1], clawClosed[1]), clawOpen[1]);
        double maxRightClaw = Math.max(Math.max(clawZero[1], clawClosed[1]), clawOpen[1]);

        leftClawControl = Range.clip(leftClawControl, minLeftClaw, maxLeftClaw);
        rightClawControl = Range.clip(rightClawControl, minRightClaw, maxRightClaw);

        leftClaw.setPosition(leftClawControl);
        rightClaw.setPosition(rightClawControl);

        baseControl = Range.clip(baseControl, 0, 1);
        elbowControl = Range.clip(elbowControl, 0, 1);

        base.setPosition(baseControl);
        elbow.setPosition(elbowControl);
    }

    void stopRobot() {
        leftPowerControl = 0;
        rightPowerControl = 0;
        leftPowerControlBack = 0;
        rightPowerControlBack = 0;

        leftPowerControl = 0;
        rightPowerControl = 0;
        leftPowerControlBack = 0;
        rightPowerControlBack = 0;

        setDrives();
    }

    void waitMillis(double millis) {

        sleep((long) millis);
//        millis = Range.clip(millis, 0.01, millis);
//        runtimeWait.reset();
//        while (runtimeWait.nanoseconds() < millis * 1000 * 1000) {
//            idle();
//        }

    }


    // ************************** ROBOT HW CLASS END  ********************************************//
}
