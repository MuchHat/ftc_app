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

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

// ************************** ROBOT HW CLASS *****************************************************//


public class Team_Hardware_V9 {

    public DigitalChannel topSwitch = null;

    // ************************** ROBOT HW VARIABLES *********************************************//
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
    double liftControl = 0;
    double leftClawControl = 0;
    double rightClawControl = 0;
    double baseControl = 0;
    double elbowControl = 0;
    double sonarMaxAdjust = 0.15;
    //********************************* CLAW POS ************************************************//

    double clawZero[] = {1.00, 0.00};
    double clawOpen[] = {0.54, 0.47};
    double clawOpenAuto[] = {0.54, 0.47};
    double clawOpenWide[] = {0.70, 0.30};
    double clawCloseLeft[] = {0.33, 0.00};
    double clawCloseRight[] = {1.00, 0.66};
    double clawClose[] = {0.38, 0.62};
    double clawCloseAuto[] = {0.38, 0.62};
    //********************************* ARM POS *************************************************//
    double armPosZero[] = {1, 0};
    // ************************** MAIN LOOP ******************************************************//
    double driveDefaultSpeed = 1.3;
    double turnDefaultSpeed = 1.0;
    double servoDefaultSpeed = 0.003;
    boolean moveLinearStopOnFlatEnabled = false;
    double moveLinearStopOnFlatRampDownInches = 3;
    DeviceInterfaceModule deviceInterface;                  // Device Object
    AnalogInput frontSonar;                // Device Object
    AnalogInput leftSonar;                // Device Object
    AnalogInput rightSonar;                // Device Object
    int targetLeft = 0;

    // ************************** HW CONSTRUCTOR  ************************************************//
    int targetRight = 0;

    // ************************** AUTO DRIVE HELPER FUNCTIONS  *********************************//
    int targetLeftBack = 0;
    int targetRightBack = 0;
    boolean moveLinearGyroTrackingEnabled = false;
    double moveLinearGyroHeadingToTrack = -1.0;

    public Team_Hardware_V9() {

    }

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
        moveLift(-1);
        moveLift(0.88);

        imuGyro.init(hwMap);
    }

    void adjustTurnTo12() {
        // correct only if heading is off > 11 deg

        if (Math.abs(gyroDrift(0)) > 11) {
            turn2Heading(0);
        }
    }

    void adjustTurnTo6() {
        // correct only if heading is off > 11 deg

        if (Math.abs(gyroDrift(180)) > 11) {
            turn2Heading(180);
        }
    }

    void adjustTurnTo3() {
        // correct only if heading is off > 11 deg

        if (Math.abs(gyroDrift(270)) > 11) {
            turn2Heading(270);
        }
    }

    void adjustTurnTo9() {
        // correct only if heading is off > 11 deg

        if (Math.abs(gyroDrift(90)) > 11) {
            turn2Heading(90);
        }
    }


    void turnTo12() {
        turn2Heading(0);
    }

    void turnTo3() {
        turn2Heading(270);
    }

    void turnTo6() {
        turn2Heading(180);
    }

    void turnTo9() {
        turn2Heading(90);
    }

    void turn2Heading(double endHeading) {

        double turnPower = 0.66;
        double turnPowerMed = 0.33;
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
        double timeOut = 8;
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

            boolean doContinue = (crrError > 1) &&
                    (turnTime.seconds() < timeOut);
            if (!doContinue) {
                break;
            }

            double crrPower = turnPower;
            if (crrError < 44) crrPower = turnPowerMed;
            if (crrError < 22) crrPower = turnPowerLow;

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

    void openClawZero() {
        leftClawControl = clawZero[0];
        rightClawControl = clawZero[1];
        setServos();
    }

    void closeClawLeft() {
        leftClawControl = clawCloseLeft[0];
        rightClawControl = clawCloseLeft[1];
        setServos();
    }

    void closeClawRight() {
        leftClawControl = clawCloseRight[0];
        rightClawControl = clawCloseRight[1];
        setServos();
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

    void moveLift(double glyphCount) {

        // moves the height of a glyph in 111 millis

        double millisToMove = Math.abs(glyphCount * 111);
        double liftPower = glyphCount > 0 ? 0.88 : -0.88;

        millisToMove = Range.clip(millisToMove, 0, 333);
        double m = 0;

        while (m < millisToMove) { //do a loop such it can stop at the switch
            liftControl = liftPower;
            setDrivesByPower();
            if (liftControl == 0) {
                break;
            }
            waitMillis(6);
            m += 6;
        }

        liftControl = 0;
        setDrivesByPower();
    }

    void move(double distanceMM) {

        moveLinear(distanceMM, 0, 0, 1.0, 1.0, 1.0, 1.0);
    }

    void moveInches(double distanceInches, double movePower, double timeOut) {

        moveLinearStopOnFlatEnabled = false;
        moveLinear(distanceInches * 24.5, movePower, timeOut, 1.0, 1.0, 1.0, 1.0);
    }

    void moveInchesStopOnFlat(double distanceInches, double movePower, double timeOut) {

        moveLinearStopOnFlatEnabled = true;
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

    // ************************** ARM  DRIVE SERVOS HELPER FUNCTIONS  ****************************//

    void moveBySonarRight(double endPos, double power, double timeOutSec) {
        moveBySonar(endPos, power, timeOutSec, SonarPosition.RIGHT);
    }

    void moveBySonar(double endPos, double movePower, double timeOutSec, Team_Hardware_V9.SonarPosition sonarPosition) {

        ElapsedTime moveTimer = new ElapsedTime();
        double minPower = 0.11; //TODO
        double rampDown = 0.08; //TODO
        double crrPower = movePower;
        double crrPos = 0;
        int prevBeaconColor = colorBeacon.getColorNumber();

        // perform the first reading, try few times to get a good reading
        for (int attempts = 0; attempts < 11; attempts++) {

            if (sonarPosition == Team_Hardware_V9.SonarPosition.FRONT) {
                crrPos = frontSonar.getVoltage();
            } else if (sonarPosition == Team_Hardware_V9.SonarPosition.LEFT) {
                crrPos = leftSonar.getVoltage();
            } else {
                crrPos = rightSonar.getVoltage();
            }
            if (Math.abs(endPos - crrPos) < sonarMaxAdjust) {
                break; // good reading
            }
            waitMillis(11);
        }

        if (Math.abs(endPos - crrPos) > sonarMaxAdjust) {
            // if error too big give up
            return;
        }

        double crrError = endPos - crrPos;
        double direction = crrError > 0 ? 1.0 : -1.0;
        moveTimer.reset();

        while (moveTimer.seconds() < timeOutSec && crrError * direction > 0.01) {

            leftDistanceControl = 0;
            rightDistanceControl = 0;
            leftDistanceControlBack = 0;
            rightDistanceControlBack = 0;

            // ORANGE MEANS MOVING BY SONAR
            colorBeacon.orange();

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

            }
            if (sonarPosition == Team_Hardware_V9.SonarPosition.LEFT) {
                leftPowerControl = -crrPower * direction;
                rightPowerControl = crrPower * direction;
                leftPowerControlBack = crrPower * direction;
                rightPowerControlBack = -crrPower * direction;
                setDrivesByPower();
            }
            if (sonarPosition == Team_Hardware_V9.SonarPosition.RIGHT) {
                leftPowerControl = crrPower * direction;
                rightPowerControl = -crrPower * direction;
                leftPowerControlBack = -crrPower * direction;
                rightPowerControlBack = crrPower * direction;
                setDrivesByPower();
            }

            waitMillis(111);
            for (int attempts = 0; attempts < 11; attempts++) {

                if (sonarPosition == Team_Hardware_V9.SonarPosition.FRONT) {
                    crrPos = frontSonar.getVoltage();
                } else if (sonarPosition == Team_Hardware_V9.SonarPosition.LEFT) {
                    crrPos = leftSonar.getVoltage();
                } else {
                    crrPos = rightSonar.getVoltage();
                }
                if (Math.abs(endPos - crrPos) < sonarMaxAdjust) {
                    break; // good reading
                }
                waitMillis(11);
            }

            if (Math.abs(endPos - crrPos) > sonarMaxAdjust) {
                break; // if no good reading give up
            }
            crrError = endPos - crrPos;
        }
        stopRobot();
        colorBeacon.colorNumber(prevBeaconColor);
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

    void moveArmPosZero() {
        moveArm(armPosZero[0], armPosZero[1]);
    }

    void moveArm(double newBase, double newElbow) {

        double stepSize = 0.006;
        double stepTime = 8;
        double rampUp = 22;
        double rampUp2 = 44;

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

            if (i > rampUp2 / 2 && i < stepCount - rampUp2) {
                i += 13;
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

    boolean isFlat() {

        boolean xFlat = imuGyro.getInclineX() < 4 || imuGyro.getInclineX() > 356; //TODO
        boolean yFlat = imuGyro.getInclineY() < 4 || imuGyro.getInclineY() > 356; //TODO

        return xFlat && yFlat;
    }

    double inchesToTarget() {


        double leftToTarget = Math.abs(targetLeft - leftDrive.getCurrentPosition());
        double rightToTarget = Math.abs(targetRight - rightDrive.getCurrentPosition());
        double leftBackToTarget = Math.abs(targetLeftBack - leftDriveBack.getCurrentPosition());
        double rightBackToTarget = Math.abs(targetRightBack - rightDriveBack.getCurrentPosition());

        return pulsesToMm((leftToTarget + rightToTarget + leftBackToTarget + rightBackToTarget) / 4) / 24.5; //TODO
    }

    double gyroDrift(double targetHeading) {

        double crrHeading = imuGyro.getHeading();

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

    void setDrivesByDistance(double timeOut) {

        ElapsedTime encodersTimer = new ElapsedTime();
        int prevBeaconColor = colorBeacon.getColorNumber();
        double lastCheckIfLocked = 0;

        if (moveLinearGyroTrackingEnabled) {
            if (moveLinearGyroHeadingToTrack < 0) {
                moveLinearGyroHeadingToTrack = imuGyro.getHeading();
            }
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        targetLeft = leftDrive.getCurrentPosition() + (int) leftDistanceControl;
        targetRight = rightDrive.getCurrentPosition() + (int) rightDistanceControl;
        targetLeftBack = leftDriveBack.getCurrentPosition() + (int) leftDistanceControlBack;
        targetRightBack = rightDriveBack.getCurrentPosition() + (int) rightDistanceControlBack;

        leftDrive.setTargetPosition(targetLeft);
        rightDrive.setTargetPosition(targetRight);
        leftDriveBack.setTargetPosition(targetLeftBack);
        rightDriveBack.setTargetPosition(targetRightBack);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftPowerControl = Range.clip(leftPowerControl, 0, 1);
        rightPowerControl = Range.clip(rightPowerControl, 0, 1);
        leftPowerControlBack = Range.clip(leftPowerControlBack, 0, 1);
        rightPowerControlBack = Range.clip(rightPowerControlBack, 0, 1);

        leftDrive.setPower(Math.abs(leftPowerControl));
        rightDrive.setPower(Math.abs(rightPowerControl));
        leftDriveBack.setPower(Math.abs(leftPowerControlBack));
        rightDriveBack.setPower(Math.abs(rightPowerControlBack));

        double timeOutSec = 4;
        if (timeOut > 0) {
            timeOutSec = timeOut;
        }
        encodersTimer.reset();

        boolean locked = false;
        double leftPrev = leftDrive.getCurrentPosition();
        double rightPrev = rightDrive.getCurrentPosition();
        double leftBackPrev = leftDriveBack.getCurrentPosition();
        double rightBackPrev = rightDriveBack.getCurrentPosition();

        // LOOP UNTIL TARGET HIT OR TIMEOUT
        while (encodersTimer.seconds() < timeOutSec) {

            // WAIT LOOP AND CHECK IF MOTORS STOPPED
            boolean stillRunning = true;
            for (int i = 0; i < 6; i++) {
                waitMillis(11);
                stillRunning = leftDrive.isBusy() ||
                        rightDrive.isBusy() ||
                        leftDriveBack.isBusy() ||
                        rightDriveBack.isBusy();
                if (!stillRunning) {
                    break;
                }
            }
            if (!stillRunning) {
                break;
            }
            // END WAIT LOOP AND CHECK IF MOTORS STOPPED
            // SLOW DOWN FOR STOP ON FLAT
            if (moveLinearStopOnFlatEnabled &&
                    (leftDrive.getPower() > 0.22 ||
                            leftDrive.getPower() > 0.22 ||
                            leftDriveBack.getPower() > 0.22 ||
                            rightDriveBack.getPower() > 0.22) &&
                    inchesToTarget() < moveLinearStopOnFlatRampDownInches) {

                leftDrive.setPower(0.22);
                rightDrive.setPower(0.22);
                leftDriveBack.setPower(0.22);
                rightDriveBack.setPower(0.22);

                // WHITE FLASH MEANS SLOW DOWN FOR RAMP DOWN
                colorBeacon.white();
            }
            // END SLOW DOWN
            // STOP ON FLAT
            if (moveLinearStopOnFlatEnabled &&
                    isFlat() &&
                    inchesToTarget() < moveLinearStopOnFlatRampDownInches) {

                // WHITE BLINK MEANS STOPPED ON FLAT
                colorBeacon.white();
                beaconBlink(1);
                stillRunning = false;
                break;
            }
            // END STOP ON FLAT
            // GYRO TRACKING IF ENABLED
            double driftRight = 0;
            if (moveLinearGyroTrackingEnabled) {
                driftRight = gyroDrift(moveLinearGyroHeadingToTrack);
            }
            if (moveLinearGyroTrackingEnabled && driftRight != 0 && stillRunning) {

                // PINK MEANS GYRO CORRECTING
                colorBeacon.pink();

                double lPower = leftDrive.getPower();
                double rPower = rightDrive.getPower();
                double lbPower = leftDriveBack.getPower();
                double rbPower = rightDriveBack.getPower();

                double direction = rightDistanceControl > 0 ? 1.0 : -1.0;

                // reduce power on one side to compensate
                // set to 0.88 for 11 deg drift
                // set to 0.11 for 90 deg drift
                double reduceRatio = 1 - 1.1 * Math.abs(driftRight);
                reduceRatio = Range.clip(reduceRatio, 0.88, 1);

                if (driftRight * direction > 0) {

                    lPower = rPower * reduceRatio; // x0.07 is the minimum for 6 deg drift
                    lbPower = rbPower * reduceRatio; // x0.15 reduction for 11 deg drift

                    if (lPower < 0.06 || lbPower < 0.06) {
                        lPower = rPower;
                        lbPower = rbPower;
                    }
                } else {

                    rPower = lPower * reduceRatio;
                    rbPower = lbPower * reduceRatio;

                    if (rPower < 0.06 || rbPower < 0.06) {
                        rPower = lPower;
                        rbPower = lbPower;
                    }
                }

                lPower = Range.clip(lPower, 0, 1);
                rPower = Range.clip(rPower, 0, 1);
                lbPower = Range.clip(lbPower, 0, 1);
                rbPower = Range.clip(rbPower, 0, 1);

                leftDrive.setPower(Math.abs(lPower));
                rightDrive.setPower(Math.abs(rPower));
                leftDriveBack.setPower(Math.abs(lbPower));
                rightDriveBack.setPower(Math.abs(rbPower));

            }
            if (moveLinearGyroTrackingEnabled && driftRight == 0 && stillRunning) {
                // not correcting
                colorBeacon.colorNumber(prevBeaconColor);

                /************************ TO ADD LATER; ADJUST TARGETS IF WHEELS SLIPPED **************

                 // set the power the same on all 4 wheels if not already
                 // needed if correction was done
                 double lPower = leftDrive.getPower();
                 double rPower = rightDrive.getPower();
                 double lbPower = leftDriveBack.getPower();
                 double rbPower = rightDriveBack.getPower();

                 double minPower = Math.min(Math.min(Math.min(lPower, rPower), lbPower), rbPower);
                 double maxPower = Math.max(Math.max(Math.max(lPower, rPower), lbPower), rbPower);

                 // rebase the targets if needed
                 int left2Target = leftDrive.getTargetPosition() - leftDrive.getCurrentPosition();
                 int right2Target = rightDrive.getTargetPosition() - leftDrive.getCurrentPosition();
                 int leftBack2Target = leftDriveBack.getTargetPosition() - leftDrive.getCurrentPosition();
                 int rightBack2Target = rightDriveBack.getTargetPosition() - leftDrive.getCurrentPosition();

                 int average2Target = (left2Target + right2Target + leftBack2Target + rightBack2Target) / 4;
                 int min2Target = Math.min(Math.min(Math.min(left2Target, right2Target), leftBack2Target), rightBack2Target);
                 int max2Target = Math.max(Math.max(Math.max(left2Target, right2Target), leftBack2Target), rightBack2Target);

                 // adjust only if a big difference
                 if (Math.abs(max2Target - min2Target) > 666) {

                 leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + average2Target);
                 rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + average2Target);
                 leftDriveBack.setTargetPosition(leftDriveBack.getCurrentPosition() + average2Target);
                 rightDriveBack.setTargetPosition(rightDriveBack.getCurrentPosition() + average2Target);
                 }
                 if (Math.abs(maxPower - minPower) > 0.08) {

                 leftDrive.setPower(Math.abs(maxPower));
                 rightDrive.setPower(Math.abs(maxPower));
                 leftDriveBack.setPower(Math.abs(maxPower));
                 rightDriveBack.setPower(Math.abs(maxPower));
                 }

                 ************** END TO ADD LATER; ADJUST TARGETS IF WHEELS SLIPPED ********************/

            }
            // END GYRO TRACKING
            // CHECK IF WHEELS LOCKED
            if (encodersTimer.milliseconds() - lastCheckIfLocked > 222) {

                lastCheckIfLocked = encodersTimer.milliseconds();

                double lMove = Math.abs(leftDrive.getCurrentPosition() - leftPrev);
                double rMove = Math.abs(rightDrive.getCurrentPosition() - rightPrev);
                double lbMove = Math.abs(leftDriveBack.getCurrentPosition() - leftBackPrev);
                double rbMove = Math.abs(rightDriveBack.getCurrentPosition() - rightBackPrev);

                double lDiffPercent = Math.abs(lMove - lbMove) / Math.max(Math.max(lMove, lbMove), 1);
                double rDiffPercent = Math.abs(rMove - rbMove) / Math.max(Math.max(rMove, rbMove), 1);
                double maxMove = Math.max(Math.max(Math.max(lMove, rMove), lbMove), rbMove);

                leftPrev = leftDrive.getCurrentPosition();
                rightPrev = rightDrive.getCurrentPosition();
                leftBackPrev = leftDriveBack.getCurrentPosition();
                rightBackPrev = rightDriveBack.getCurrentPosition();

                // diff is bigger than 33%
                // do not check for lock if moving slowly at the end
                locked = (Math.abs(lMove - lbMove) > 222 || Math.abs(rMove - rbMove) > 222); // TODO

                //if one wheel is locked stop and restart,
                //check for stall 222ms after start not sooner
                if (locked && encodersTimer.seconds() > 0.22) {

                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    leftDriveBack.setPower(0);
                    rightDriveBack.setPower(0);

                    // YELLOW MEANS LOCKED UP
                    colorBeacon.yellow();
                    beaconBlink(4); // takes care of wait too

                    leftDrive.setPower(Math.abs(leftPowerControl));
                    rightDrive.setPower(Math.abs(rightPowerControl));
                    leftDriveBack.setPower(Math.abs(leftPowerControlBack));
                    rightDriveBack.setPower(Math.abs(rightPowerControlBack));
                }
                if (!locked) {
                    colorBeacon.colorNumber(prevBeaconColor);
                }
            }
            //END CHECK IF LOCKED
        }
        // END LOOP UNTIL TARGET HIT OR TIMEOUT

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (moveLinearGyroTrackingEnabled) {
            moveLinearGyroTrackingEnabled = false;
            moveLinearGyroHeadingToTrack = -1.0;
            colorBeacon.colorNumber(prevBeaconColor);
        }
        if (moveLinearStopOnFlatEnabled) {
            moveLinearStopOnFlatEnabled = false;
            colorBeacon.colorNumber(prevBeaconColor);
        }
    }

    // ************************** HARDWARE SET FUNCTIONS *****************************************//

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

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (leftPowerControl >= 0) {
            leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            leftDrive.setPower(Math.abs(leftPowerControl));
        }
        if (leftPowerControl < 0) {
            leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            leftDrive.setPower(Math.abs(leftPowerControl));
        }
        if (rightPowerControl >= 0) {
            rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightDrive.setPower(Math.abs(rightPowerControl));
        }
        if (rightPowerControl < 0) {
            rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rightDrive.setPower(Math.abs(rightPowerControl));
        }
        if (leftPowerControlBack >= 0) {
            leftDriveBack.setDirection(DcMotorSimple.Direction.REVERSE);
            leftDriveBack.setPower(Math.abs(leftPowerControlBack));
        }
        if (leftPowerControlBack < 0) {
            leftDriveBack.setDirection(DcMotorSimple.Direction.FORWARD);
            leftDriveBack.setPower(Math.abs(leftPowerControlBack));
        }
        if (rightPowerControlBack >= 0) {
            rightDriveBack.setDirection(DcMotorSimple.Direction.FORWARD);
            rightDriveBack.setPower(Math.abs(rightPowerControlBack));
        }
        if (rightPowerControlBack < 0) {
            rightDriveBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightDriveBack.setPower(Math.abs(rightPowerControlBack));
        }
    }

    void setServos() {

        leftClawControl = Range.clip(leftClawControl, 0.13, 1);
        rightClawControl = Range.clip(rightClawControl, 0, 0.87);

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
            waitMillis(33);
            colorBeacon.colorNumber(color);
        }
    }

    void showTeamColor() {
        if (blueTeam)
            colorBeacon.blue();
        else
            colorBeacon.red();
    }

    void waitMillis(double millis) {
        sleep((long) millis);
    }

    public enum SonarPosition {
        FRONT,
        LEFT,
        RIGHT
    }


// ************************** ROBOT HW CLASS END  ********************************************//
}
