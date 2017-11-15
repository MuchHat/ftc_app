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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

// ************************** ROBOT HW CLASS *****************************************************//



public class Team_Hardware_V9 {

    public enum SonarPosition{
        FRONT,
        LEFT,
        RIGHT
    }

    // ************************** ROBOT HW VARIABLES *********************************************//

    public DigitalChannel topSwitch = null;
    public DigitalChannel bottomSwitch = null;
    public ColorSensor colorSensor = null;
    public HardwareMap hwMap = null;
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor rightDriveBack = null;
    public DcMotor leftDriveBack = null;
    public DcMotor liftDrive = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public Servo base = null;
    public Servo elbow = null;

    public MRIColorBeacon colorBeacon;
    public ImuGyro imuGyro;

    //********************************* FIELD ***************************************************//
    boolean blueTeam = true;
    boolean shortField = true;

    // ********************************* MOVE STATES **********************************************//
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

    //********************************* CLAW POS ************************************************//
    double clawClose[] = {0.42, 0.63};
    double clawZero[] = {1.00, 0.00};
    double clawOpen[] = {0.53, 0.51};
    double clawOpenWide[] = {0.69, 0.30};
    double clawCloseAuto[] = {0.42, 0.60};
    double clawOpenAuto[] = {0.53, 0.51};

    //********************************* ARM POS *************************************************//
    double armPosZero[] = {1, 0};

    // ************************** MAIN LOOP ******************************************************//
    double driveDefaultSpeed = 1.3;
    double turnDefaultSpeed = 1.0;
    double servoDefaultSpeed = 0.003;

    DeviceInterfaceModule deviceInterface;                  // Device Object
    AnalogInput frontSonar;                // Device Object
    AnalogInput leftSonar;                // Device Object
    AnalogInput rightSonar;                // Device Object

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
        imuGyro = new ImuGyro();

        colorBeacon = new MRIColorBeacon();
        colorBeacon.init(hwMap, "Beacon");

        frontSonar = hwMap.get(AnalogInput.class, "Front_Sonar");
        leftSonar = hwMap.get(AnalogInput.class, "Left_Sonar");
        rightSonar = hwMap.get(AnalogInput.class, "Right_Sonar");

        colorSensor.enableLed(false);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        liftDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
        liftDrive.setPower(0);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftPowerControl = 0;
        rightPowerControl = 0;
        leftPowerControlBack = 0;
        rightPowerControlBack = 0;
        leftClawControl = clawCloseAuto[0];
        rightClawControl = clawCloseAuto[1];
        baseControl = armPosZero[0];
        elbowControl = armPosZero[1];
        setDrivesByPower();
        setServos();

        imuGyro.init(hwMap);
    }

    // ************************** MANUAL DRIVE HELPER FUNCTIONS  *********************************//

    void turnTo12() {
        turn2Heading(0);
        turn2Heading(0);
    }

    void turnTo3() {
        turn2Heading(270);
        turn2Heading(270);
    }

    void turnTo6() {
        turn2Heading(180);
        turn2Heading(180);
    }

    void turnTo9() {
        turn2Heading(90);
        turn2Heading(90);
    }

    void turn2Heading(double endHeading) {

        double turnPower = 0.88;
        double turnPowerMed = 0.66;
        double turnPowerLow = 0.11;
        int prevBeaconColor = colorBeacon.getColorNumber();

        colorBeacon.teal();
        double startHeading = imuGyro.getHeading();

        double diffAbs = Math.abs(endHeading - startHeading);
        double diffAbs360 = 360 - diffAbs;

        double direction = endHeading > startHeading ? 1.0 : -1.0;

        if (diffAbs360 < diffAbs) {
            direction *= -1;
        }

        if (startHeading >= 180 && direction > 0 && endHeading < 180) {
            endHeading += 360;
        }
        if (startHeading <= 180 && direction < 0 && endHeading > 180) {
            endHeading -= 360;
        }

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (direction > 0) {
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
            rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        } else {
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
            leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
            rightDriveBack.setDirection(DcMotor.Direction.REVERSE);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);

        ElapsedTime turnTime = new ElapsedTime();
        turnTime.reset();
        double timeOut = 30;
        double crrError = 0;

        while (true) {

            double crrHeading = imuGyro.getHeading();

            if (startHeading >= 180 && direction > 0 && crrHeading < 180) {
                crrHeading += 360;
            }
            if (startHeading <= 180 && direction < 0 && crrHeading > 180) {
                crrHeading -= 360;
            }
            crrError = (endHeading - crrHeading) * direction;

            boolean doContinue = (crrError > 0) &&
                    (turnTime.seconds() < timeOut);
            if (!doContinue) {
                break;
            }

            double crrPower = turnPower;
            if (crrError < 13) crrPower = turnPowerMed;
            if (crrError < 9) crrPower = turnPowerLow;

            leftDrive.setPower(Math.abs(crrPower));
            rightDrive.setPower(Math.abs(crrPower));
            leftDriveBack.setPower(Math.abs(crrPower));
            rightDriveBack.setPower(Math.abs(crrPower));

            sleep(111);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);

        colorBeacon.colorNumber(prevBeaconColor);
    }

    void openClaw() {
        leftClawControl = clawOpen[0];
        rightClawControl = clawOpen[1];
        setServos();
    }

    void openClawWide() {
        leftClawControl = clawOpenWide[0];
        rightClawControl = clawOpenWide[1];
        setServos();
    }

    void closeClaw() {
        leftClawControl = clawClose[0];
        rightClawControl = clawClose[1];
        setServos();
    }

    void closeClawAuto() {
        leftClawControl = clawCloseAuto[0];
        rightClawControl = clawCloseAuto[1];
        setServos();
    }

    void openClawAuto() {
        leftClawControl = clawOpenAuto[0];
        rightClawControl = clawOpenAuto[1];
        setServos();
    }

    boolean isClawOpened() {
        return (leftClawControl == clawOpen[0] &&
                rightClawControl == clawOpen[1]);
    }

    void moveLift(double distance) {
        double mmMillis = 0.03;
        double stepTime = Math.abs(distance) / mmMillis;
        double liftDefaultPower = distance > 0 ? 1.0 : -1.0;

        stepTime = Range.clip(stepTime, 0, 3333);

        for (double i = 0; i < stepTime; i++) { //do a loop such it can stop at the switch
            liftControl = liftDefaultPower;
            setDrivesByPower();
            waitMillis(33);
        }

        liftControl = 0;
        setDrivesByPower();
    }

    void move(double distanceMM) {

        moveLinear(distanceMM, 0, 0, 1.0, 1.0, 1.0, 1.0);
    }

    void moveInches(double distanceInches, double movePower, double timeOut) {


        moveLinear(distanceInches * 24.5, movePower, timeOut, 1.0, 1.0, 1.0, 1.0);
    }

    void moveSideInches(double distanceInches, double movePower, double timeOut) {

        double moveSidePower = 0.44;
        if (moveSidePower > 0) moveSidePower = movePower;
        moveLinear(distanceInches * 24.5, moveSidePower, timeOut, 1.0, -1.0, -1.0, 1.0);
    }

    void moveBySonarFront(double endPos, double power, double timeOutSec) {
        moveBySonar(endPos, power, timeOutSec, SonarPosition.FRONT);
    }

    void moveBySonarLeft(double endPos, double power, double timeOutSec) {
        moveBySonar(endPos, power, timeOutSec, SonarPosition.LEFT);
    }

    void moveBySonarRight(double endPos, double power, double timeOutSec) {
        moveBySonar(endPos, power, timeOutSec, SonarPosition.RIGHT);
    }

    void moveBySonar(double endPos, double movePower, double timeOutSec, Team_Hardware_V9.SonarPosition sonarPosition) {

        ElapsedTime moveTimer = new ElapsedTime();
        double minPower = 0.22; //TODO
        double rampDown = 0.08; //TODO
        double crrPower = movePower;
        double crrPos = 0;

        if (sonarPosition == Team_Hardware_V9.SonarPosition.FRONT) {
            crrPos = frontSonar.getVoltage();
        } else if (sonarPosition == Team_Hardware_V9.SonarPosition.LEFT) {
            crrPos = leftSonar.getVoltage();
        } else {
            crrPos = rightSonar.getVoltage();
        }

        double crrError = endPos - crrPos;
        double direction = crrError > 0 ? 1.0 : -1.0;
        moveTimer.reset();

        while (moveTimer.seconds() < timeOutSec && crrError * direction > 0.01) {

            leftDistanceControl = 0;
            rightDistanceControl = 0;
            leftDistanceControlBack = 0;
            rightDistanceControlBack = 0;

            crrPower = movePower;
            if (crrError * direction < rampDown) {
                crrPower = minPower;
            }

            if (sonarPosition == Team_Hardware_V9.SonarPosition.FRONT) {
                leftPowerControl = -crrPower * direction;
                rightPowerControl = -crrPower * direction;
                leftPowerControlBack = -crrPower * direction;
                rightPowerControlBack = -crrPower * direction;
                setDrivesByPower();

            } else if( sonarPosition == Team_Hardware_V9.SonarPosition.LEFT){
                leftPowerControl = -crrPower * direction;
                rightPowerControl = crrPower * direction;
                leftPowerControlBack = crrPower * direction;
                rightPowerControlBack = -crrPower * direction;
                setDrivesByPower();

            } else {
                leftPowerControl = crrPower * direction;
                rightPowerControl = -crrPower * direction;
                leftPowerControlBack = -crrPower * direction;
                rightPowerControlBack = crrPower * direction;
                setDrivesByPower();
            }

            waitMillis(111);
            if (sonarPosition == Team_Hardware_V9.SonarPosition.FRONT) {
                crrPos = frontSonar.getVoltage();
            } else if (sonarPosition == Team_Hardware_V9.SonarPosition.LEFT) {
                crrPos = leftSonar.getVoltage();
            } else {
                crrPos = rightSonar.getVoltage();
            }
            crrError = endPos - crrPos;
        }
        stopRobot();
    }



    void moveLinear(double distanceMM, double power, double timeOut, double dirFrontLeft, double dirFrontRight, double dirBackLeft, double dirBackRight) {

        double distanceLowSpeedMM = 99;

        double lowSpeedPower = 0.66;
        double highSpeedPower = 1.0;

        if (power != 0) {
            lowSpeedPower = Math.abs(power);
            highSpeedPower = Math.abs(power);
        }

        //convert from mm to tics
        double correction = 4.75;
        double revolutions = distanceMM / ((25.4 * 4) * Math.PI); //a 4 inch wheel
        double distancePulses = revolutions * (7 * 60) * correction; // 420 tics per revolution

        double crrPower = Math.abs(distanceMM) > distanceLowSpeedMM ? highSpeedPower : lowSpeedPower;

        leftPowerControl = crrPower;
        rightPowerControl = crrPower;
        leftPowerControlBack = crrPower;
        rightPowerControlBack = crrPower;

        leftDistanceControl = distancePulses * dirFrontLeft;
        rightDistanceControl = distancePulses * dirFrontRight;
        leftDistanceControlBack = distancePulses * dirBackLeft;
        rightDistanceControlBack = distancePulses * dirBackRight;

        setDrivesByDistance(timeOut);
        stopRobot();
    }

    // ************************** ARM  DRIVE SERVOS HELPER FUNCTIONS  ****************************//

    void moveArmPosZero() {
        moveArm(armPosZero[0], armPosZero[1]);
    }

    void moveArm(double newBase, double newElbow) {

        double stepSize = 0.006;
        double stepTime = 8;
        double rampUp = 33;
        double rampUp2 = 66;

        double baseStart = baseControl;
        double elbowStart = elbowControl;

        double stepCount = Math.abs(baseStart - newBase) / stepSize;
        stepCount = Math.max(Math.abs(elbowStart - newElbow) / stepSize, stepCount);
        stepCount = Range.clip(stepCount, 0, 333);

        double elbowStepSize = (newElbow - elbowStart) / stepCount;
        double baseStepSize = (newBase - baseStart) / stepCount;

        int i = 0;
        while (i < (int) stepCount) {

            double crrStepTime = stepTime;

            if (i > rampUp2 / 3 && i < stepCount - rampUp2) {
                i += 7;
            } else if (i > rampUp / 5 && i < stepCount - rampUp) {
                i += 3;
            } else {
                i++;
            }
            double baseCrr = baseStart + i * baseStepSize;
            double elbowCrr = elbowStart + i * elbowStepSize;

            baseControl = baseCrr;
            elbowControl = elbowCrr;
            setServos();

            waitMillis(crrStepTime);
        }

        baseControl = newBase;
        elbowControl = newElbow;
        setServos();
    }

    // ************************** HARDWARE SET FUNCTIONS *****************************************//

    void setDrivesByDistance(double timeOut) {

        ElapsedTime encodersTimer = new ElapsedTime();

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) leftDistanceControl);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) rightDistanceControl);
        leftDriveBack.setTargetPosition(leftDriveBack.getCurrentPosition() + (int) leftDistanceControlBack);
        rightDriveBack.setTargetPosition(rightDriveBack.getCurrentPosition() + (int) rightDistanceControlBack);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(Math.abs(leftPowerControl));
        rightDrive.setPower(Math.abs(rightPowerControl));
        leftDriveBack.setPower(Math.abs(leftPowerControlBack));
        rightDriveBack.setPower(Math.abs(rightPowerControlBack));

        double timeOutSec = 4;
        if (timeOut > 0) {
            timeOutSec = timeOut;
        }
        encodersTimer.reset();

        while (encodersTimer.seconds() < timeOutSec) {

            boolean stillRunning = leftDrive.isBusy() ||
                    rightDrive.isBusy() ||
                    leftDriveBack.isBusy() ||
                    rightDriveBack.isBusy();
            if (!stillRunning) {
                break;
            }
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        double headingCorrection = 0;

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

    void setClawPosZero() {
        leftClawControl = clawZero[0];
        rightClawControl = clawZero[1];
        setServos();
    }

    void setServos() {

        leftClawControl = Range.clip(leftClawControl, 0, 1);
        rightClawControl = Range.clip(rightClawControl, 0, 1);

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

        rightDrive.setPower(0);
        leftDrive.setPower(0);
        rightDriveBack.setPower(0);
        leftDriveBack.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void beaconBlink(int count) {
        int color = colorBeacon.getColorNumber();

        for (int i = 0; i < count; i++) {
            colorBeacon.off();
            waitMillis(333);
            colorBeacon.colorNumber(color);
        }
    }

    void showTeamColor() {
        if (blueTeam)
            colorBeacon.blue();
        else
            colorBeacon.red();
    }

    void showTeamColorLight() {
        if (blueTeam)
            colorBeacon.lightBlue();
        else
            colorBeacon.lightRed();
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
