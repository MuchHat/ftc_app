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

//********************************* OP CLASS*** **************************************************//

@TeleOp(name = "Team V4", group = "Team")
// @Disabled
public class Team_OpMode_V4 extends LinearOpMode {

    //********************************* HW VARIABLES *********************************************//

    public Team_Hardware_V2 robot = new Team_Hardware_V2();

    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    private IntegratingGyroscope gyro;

    private ElapsedTime runtimeLoop = new ElapsedTime();

    //********************************* MOVE STATE ***********************************************//

    private double leftDriveControl = 0;
    private double rightDriveControl = 0;
    private double headingControl = 0;
    private double liftControl = 0;
    private double leftClawControl = 0;
    private double rightClawControl = 0;

    private double baseControl = 0;
    private double elbowControl = 0;
    private double gameStartHeading = 0;

    private Boolean armEnabled = false;

    private Boolean manualMode = true;
    private Boolean blueTeam = true;
    private Boolean rightField = true;

    //********************************* CONSTANTS ************************************************//

    private double driveDefaultSpeed = 0.44; // TODO
    private double turnDefaultSpeed = 0.22; // TODO
    private double liftDefaultSpeed = 0.22; // TODO
    private double servoDefaultSpeed = 0.00033; // TODO

    //********************************* PREDEF POS ***********************************************//

    private double clawOpen[] = {0.76, 0.44};
    private double clawClosed[] = {0.85, 0.34};

    @Override
    public void runOpMode() {

        //********************************* INIT LOOP ********************************************//
        robot.init(hardwareMap);

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = modernRoboticsI2cGyro;

        telemetry.log().add("calibrating gyro, do not move");
        sleep(444);
        modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        runtimeLoop.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating gyro", "%s",
                    Math.round(runtimeLoop.seconds()) % 2 == 0 ? "..  " : "   ..");
            telemetry.update();
            sleep(66);
        }
        telemetry.log().clear();

        while (!isStopRequested() && !gamepad1.start) {

            String mode = manualMode ? "manual" : "autonomous";
            String field = rightField ? "right" : "left";
            String team = blueTeam ? "blue" : "red";

            telemetry.log().add("select mode");
            telemetry.log().add("(a) for manual or (b) for autonomous: " + mode);
            telemetry.log().add("(x) for blue tem or (y) for red team: " + team);
            telemetry.log().add("(r bumper) for right field or (l bumper) for left field: ", field);
            telemetry.log().add("press start to continue");
            telemetry.update();

            if (gamepad1.a) manualMode = true;
            if (gamepad1.b) manualMode = false;

            if (gamepad1.x) blueTeam = true;
            if (gamepad1.y) blueTeam = false;

            if (gamepad1.right_bumper) rightField = true;
            if (gamepad1.left_bumper) rightField = false;
        }

        setDrives();
        setServos();

        updateTelemetry();

        waitForStart();
        runtimeLoop.reset();

        while (opModeIsActive()) {

            //********************************* START LOOP *****************************************

            double crrLoopTime = runtimeLoop.nanoseconds() / 1000000; // covert to millis
            runtimeLoop.reset();

            updateTelemetry();

            //********************************* MANUAL MODE *****************************************

            if (manualMode) {

                // control: DRIVES
                {
                    double xInput = 0;
                    double yInput = 0;

                    if (Math.abs(gamepad1.left_stick_x) > 0.15)
                        xInput = gamepad1.left_stick_x; //TODO
                    if (Math.abs(gamepad1.left_stick_y) > 0.15)
                        yInput = -gamepad1.left_stick_y; //TODO

                    leftDriveControl = yInput * driveDefaultSpeed;
                    rightDriveControl = yInput * driveDefaultSpeed;

                    leftDriveControl += xInput * turnDefaultSpeed;
                    rightDriveControl -= xInput * turnDefaultSpeed;

                    setDrives();

                    if (xInput != 0) headingControl = modernRoboticsI2cGyro.getHeading();
                }

                // control: LIFT
                {
                    double liftInput = 0;

                    if (Math.abs(gamepad1.right_stick_y) > 0.15)
                        liftInput = gamepad1.right_stick_y; //TODO

                    liftControl = liftInput * liftDefaultSpeed;

                    setDrives();
                }

                // control: TURNS 90
                if (gamepad1.dpad_right) {
                    doTurn(90);
                }

                // control: TURNS 90
                if (gamepad1.dpad_left) {
                    doTurn(-90);
                }

                // control: TURNS 90
                if (gamepad1.dpad_left) {
                    doTurn(180);
                }

                // control: TURNS 90
                if (gamepad1.start) {
                    turnToHeading(gameStartHeading);
                }

                // control: CLAW OPEN
                if (gamepad1.left_trigger != 0) {
                    leftClawControl -= gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                    rightClawControl += gamepad1.left_trigger * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }

                // control: CLAW CLOSE
                if (gamepad1.right_trigger != 0) {
                    leftClawControl += gamepad1.right_trigger * servoDefaultSpeed * crrLoopTime;
                    rightClawControl -= gamepad1.right_trigger * servoDefaultSpeed * crrLoopTime;
                    setServos();
                }
                // control: CLAW PREDEF OPEN
                if (gamepad1.left_bumper) {
                    leftClawControl = clawOpen[0];
                    rightClawControl = clawOpen[1];
                    setServos();
                }
                // control: CLAW PREDEF CLOSE
                if (gamepad1.right_bumper) {
                    leftClawControl = clawClosed[0];
                    rightClawControl = clawClosed[1];
                    setServos();
                }
            }

            //********************************* AUTO MODE *****************************************

            if (!manualMode) {
                runAutonomous();
            }

            //********************************* END LOOP *****************************************//
        }
    }

    // ************************** AUTO MODE ******************************************************//

    private void runAutonomous() {

        //example
        //TODO

        moveStraight(200);
        doTurn(90);
        waitMillis(555);

        moveStraight(200);
        doTurn(-90);
        waitMillis(555);

        moveStraight(400);
        doTurn(180);
        waitMillis(555);

        moveStraight(200);
        doTurn(-90);
        waitMillis(555);

        turnToHeading(gameStartHeading);

        manualMode = false; // stop autonomous
    }

    // ************************** MOVE HELPER FUNCTIONS  *****************************************//

    private void turnToHeading(double newHeading) {

        newHeading %= 360;

        double crrHeading = modernRoboticsI2cGyro.getHeading();
        double turnDeg = newHeading - crrHeading;

        if (Math.abs(newHeading - crrHeading) > 360 - Math.abs(newHeading - crrHeading)) {
            double direction = newHeading > crrHeading ? 1.0 : -1.0;

            turnDeg = 360 - Math.abs(newHeading - crrHeading);
            turnDeg *= direction;
        }

        doTurn(turnDeg);
    }

    private void doTurn(double turnDeg) {

        turnDeg %= 360;
        if (turnDeg > 180) turnDeg = turnDeg - 360;
        else if (turnDeg < -180) turnDeg = turnDeg + 360;
        turnDeg %= 180;

        double startHeading = modernRoboticsI2cGyro.getHeading();
        double endHeading = startHeading + turnDeg;
        double direction = turnDeg > 0 ? 1.0 : -1.0;

        double error = Math.abs(endHeading - startHeading) / 180;
        error = Range.clip(error, 0.0, 0.66); // the bigger the error the more off

        double iterations = 0;

        while (error > 0.05 && iterations < 999) { //TODO
            double turnPower = 0.2; //TODO

            turnPower *= (1 - error);
            turnPower = Range.clip(turnPower, 0.05, 0.2); //TODO

            leftDriveControl = -turnPower * direction;
            rightDriveControl = turnPower * direction;
            setDrives();

            waitMillis(5); //TODO

            double crrHeading = modernRoboticsI2cGyro.getHeading();

            if (startHeading >= 180 && crrHeading <= 180 && direction > 0) {
                crrHeading += 360;
            }
            if (startHeading < 180 && crrHeading > 180 && direction < 0) {
                crrHeading -= 360;
            }

            error = Math.abs(endHeading - crrHeading) / 180;
            error = Range.clip(error, 0.0, 0.66);

            iterations++;
        }
        leftDriveControl = 0;
        rightDriveControl = 0;
        setDrives();

        headingControl = modernRoboticsI2cGyro.getHeading();
    }

    private void moveStraight(double mmDistance) {

        //time based: 50 cycles at 5 millis at 0.2 power does a 5mm move
        double totalSteps = mmDistance * 10;

        for (int i = 0; i < totalSteps; i++) {

            double error = 1;
            if (i < totalSteps * 0.33) error = (totalSteps * 0.33 - i) / totalSteps; //TODO
            if (i > totalSteps * 0.66) error = (totalSteps - i) / totalSteps; //TODO
            error = Range.clip(Math.abs(error), 0, 0.66); // 2mm to 6mm ramp //TODO

            leftDriveControl = 0.2 * (1 - error); //TODO
            rightDriveControl = 0.2 * (1 - error); //TODO
            setDrives();
            waitMillis(5);
        }

        leftDriveControl = 0;
        rightDriveControl = 0;
        setDrives();
    }

    void moveArm(double newBase, double newElbow) {

        if (!armEnabled) {
            return;
        }

        newBase = Range.clip(newBase, 0.05, 0.95); //TODO
        newElbow = Range.clip(newElbow, 0.05, 0.95); //TODO

        double maxServoStep = 0.004; // 0.1 per servo and step
        double stepCount = 0;
        stepCount = Math.max(stepCount, Math.abs(newElbow - elbowControl) / maxServoStep);
        stepCount = Math.max(stepCount, Math.abs(newBase - baseControl) / maxServoStep);

        if (stepCount > 0) {
            // move the elbow first to avoid hitting the robot

            ElapsedTime stepElapsedTime = new ElapsedTime();
            stepElapsedTime.reset();

            for (int i = 0; i < stepCount; i++) {

                double baseControlStep = baseControl + i * (newBase - baseControl) / stepCount;
                double elbowControlStep = elbowControl + i * (newElbow - elbowControl) / stepCount;

                elbowControlStep = Range.clip(elbowControlStep, 0.05, 0.95); //TODO
                baseControlStep = Range.clip(baseControlStep, 0.05, 0.95); //TODO

                while (stepElapsedTime.milliseconds() < 3 * (i + 1)) {
                    idle();
                }
                robot.elbow.setPosition(elbowControlStep);
                robot.base.setPosition(baseControlStep);
            }
        }

        robot.elbow.setPosition(newElbow);
        robot.base.setPosition(newBase);

        baseControl = newBase;
        elbowControl = newElbow;

    }

    // ************************** HARDWARE SET FUNCTIONS *****************************************//

    void setDrives() {

        leftDriveControl = Range.clip(leftDriveControl, -0.66, 0.66); //TODO max max power
        rightDriveControl = Range.clip(rightDriveControl, -0.66, 0.66); //TODO max max power

        liftControl = Range.clip(liftControl, -0.66, 0.66); //TODO max max power

        if (liftControl >= 0 && !robot.topSwitch.getState()) { // false means switch is pressed
            liftControl = 0;
        }
        if (liftControl < 0 && !robot.bottomSwitch.getState()) { // false means switch is pressed
            liftControl = 0;
        }
        if (liftControl < 0) {
            robot.liftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.liftDrive.setPower(-liftControl);
        }
        if (liftControl >= 0) {
            robot.liftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.liftDrive.setPower(liftControl);
        }

        // if gyro indicates drifting add same power to correct
        // do not add if already doing a turn
        double headingCorrection = 0;

        if (leftDriveControl != rightDriveControl) {
            double error = modernRoboticsI2cGyro.getHeading() - headingControl;

            if (error > 180) error = -360 + error; // convert to +/- 180
            headingCorrection = error / 180; //TODO tune up the amount of correction
        }


        if (leftDriveControl >= 0) {
            robot.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.leftDrive.setPower(leftDriveControl - headingCorrection);
        }
        if (leftDriveControl < 0) {
            robot.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.leftDrive.setPower(-leftDriveControl - headingCorrection);
                // apply the correction the oposite way if going reverse //TODO
        }
        if (rightDriveControl >= 0) {
            robot.rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.rightDrive.setPower(rightDriveControl + headingCorrection);
        }
        if (rightDriveControl < 0) {
            robot.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.rightDrive.setPower(-rightDriveControl + headingCorrection);
                // apply the correction the oposite way if going reverse //TODO
        }

    }

    void setServos() {

        leftClawControl = Range.clip(leftClawControl, 0.05, 0.95); //TODO
        rightClawControl = Range.clip(rightClawControl, 0.05, 0.95); //TODO

        robot.leftClaw.setPosition(leftClawControl);
        robot.rightClaw.setPosition(rightClawControl);
    }

    // ************************** HELPER FUNCTIONS ***********************************************//

    private void updateTelemetry() {

        String mode = manualMode ? "manual" : "autonomous";
        String field = rightField ? "right" : "left";
        String team = blueTeam ? "blue" : "red";

        telemetry.addData("left drive", "%.0f%%", leftDriveControl * 100);
        telemetry.addData("right drive", "%.0f%%", rightDriveControl * 100);
        telemetry.addData("lift", "%.0f%%", liftControl * 100);
        telemetry.addData("left claw", "%.0f%%", leftClawControl * 100);
        telemetry.addData("right claw", "%.0f%%", rightClawControl * 100);
        telemetry.addData("crr heading", "%.0fdeg", (double) modernRoboticsI2cGyro.getHeading());
        telemetry.addData("set heading", "%.0fdeg", (double) headingControl);
        telemetry.addData("start heading", "%.0fdeg", gameStartHeading);
        telemetry.addData("z value", "%.0fdeg", (double)modernRoboticsI2cGyro.getIntegratedZValue());
        telemetry.addData("mode", mode);
        telemetry.addData("team", team);
        telemetry.addData("field", field);

        telemetry.update();

    }

    private void waitMillis(double millis) {
        ElapsedTime runtimeWait = new ElapsedTime();

        runtimeWait.reset();

        while (runtimeWait.milliseconds() < millis) {
            idle();
        }
    }

    // ************************** OP END *********************************************************//
}

