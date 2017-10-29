package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/* NOTES

https://github.com/ftctechnh/ftc_app/wiki/Identifying-Vuforia-VuMarks

Vuforia key

Ac6cr63/////AAAAGUsQTEyyG0kggwF13U8WoMlPgXZiUoKR9pf2nlfhVVfvDXFsTn0wufo
ywxzibq+y5BGBa2cChKWAcUkKaD9/ak5lwCm9Wp3Osk9omsMR0YYoxt4TuPktrflK4HuTH
8cOAQA8YDuOs/SO/cgOmWbQZtRXN/lFkUwGZA9eiV5D8730BG2SBLPR4A9rcFs0Fp/yPgc
m4Zsh5Kv2Ct8XjJXmXk5mAjERZ5B6hKQzf/4wd9tSQ6BeQLvsgd5nI0Pj+K1NHI4EyHdFy
xCPu91AMcCsXCLjkABfYt11Zhxu1uYaFF/AcN3eBHRwprVpDEBBXOMnD4BRCj0xxYYPWWO6
g4gcjqBPgBos5nCDk43KipEeX22z

 */

//********************************* MAIN OP CLASS ************************************************//

@TeleOp(name = "Auto Blue Short Safe V9", group = "Competition")
// @Disabled
public class Auto_Blue_Short_Safe_V9 extends LinearOpMode {

    //********************************* HW VARIABLES *********************************************//
    private Team_Hardware_V9 robot = new Team_Hardware_V9();

    //********************************* MOVE STATES **********************************************//
    private ElapsedTime totalRuntime = null;

    private Boolean blueTeam = true;
    private Boolean shortField = true;
    private Boolean loaded = false;

    private Vu vu = new Vu();

    // ************************** MAIN LOOP ******************************************************//
    @Override
    public void runOpMode() {

        //********************************* MAIN LOOP INIT ***************************************//
        robot.init(hardwareMap);

        ElapsedTime controlRuntime = new ElapsedTime();
        ElapsedTime loopRuntime = new ElapsedTime();
        totalRuntime = new ElapsedTime();

        controlRuntime.reset();
        if (blueTeam) robot.colorBeacon.blue();
        if (!blueTeam) robot.colorBeacon.red();

        robot.moveArm(robot.armPosZero[0], robot.armPosZero[1]);

        robot.modernRoboticsI2cGyro.calibrate();
        // Wait until the gyro calibration is complete
        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating gyro", "... do NOT move");
            telemetry.addData("calibrating gyro", "%s", Math.round(controlRuntime.seconds()));
            loaded = true;
            telemetry.update();
            sleep(66);
        }

        telemetry.clear();
        telemetry.update();
        telemetry.addData("driver", "CLICK  >>> to START");
        telemetry.update();

        waitForStart();

        vu.init(hardwareMap);

        while (opModeIsActive() && loaded) {


            double crrLoopTime = loopRuntime.nanoseconds() / 1000000; // covert to millis
            loopRuntime.reset();
            updateTelemetry();

            //********************************* AUTO MOVE TO THE SAFE ZONE ***********************//

            robot.colorBeacon.yellow();
            robot.move(120);
            waitMillis(111);
            robot.move(-60);

            // get using encoders in the general area

            // load vuforia, turn green if marker found
            waitMillis(111);
            if (vu.targetSeen()) {

                robot.colorBeacon.green();

                int errDis = 50; // 1 left 2 center 3 right

                int[] xValues = {540, 465, 450}; // Desired value for left, right, and middle
                int desiredX = xValues[vu.getLastTargetSeenNo() - 1];

                waitMillis(333);
                robot.move(50);

                double vuX = vu.getX();
                double attempts = 0;

                while (vuX < desiredX && attempts < 8) {
                    robot.colorBeacon.purple();
                    vuX = vu.getX();
                    if (desiredX - vuX < 100) {
                        robot.move(2);
                        waitMillis(33);

                    } else {
                        robot.move(7);
                        waitMillis(33);

                    }
                    attempts++;
                }
                if( blueTeam)robot.colorBeacon.blue();
                else robot.colorBeacon.red();

                if (vu.getX() - desiredX > errDis) {
                    robot.move(-2);
                }

            } else {
                // turn yellow if not found
                waitMillis(333);
                robot.colorBeacon.yellow();
            }

            //turn to put the glyph in
            robot.turn(90);

            //put the glyph in
            robot.move(11);

            robot.openClaw();
            waitMillis(111);

            robot.move(-11);

            robot.stopRobot();

            stop(); //stop the opMode

            //********************************* END LOOP *****************************************//
        }
    }

    void vuAdjust(double positionDesiredX, double positionDesiredY, double moveSide) {

        if (!vu.targetSeen()) {
            // vuforia not working
            return;
        }

        double tolerance = 222; //in vu units

        boolean useX = false;
        boolean useY = false;

        //use only one X or Y, the one matching the current orientation of the robot
        if (positionDesiredX != 0) useX = true;
        else if (positionDesiredY != 0) useY = true;

        double positionDesired = 0;
        if (useX) positionDesired = positionDesiredX;
        if (useY) positionDesired = positionDesiredY;

        double positionActual = 0;
        if (useX) positionActual = vu.getX();
        if (useY) positionActual = vu.getY();

        double dir = positionDesired > positionActual ? 1.0 : -1.0;
        double err = positionDesired - positionActual;

        if (err * dir > tolerance) {
            // the diff is too big, vuforia is probably not working
            return;
        }

        int attempts = 0;
        while (err * dir > 0 && attempts < 6) {
            double step = 4; //move 4 mm at a time

            if (moveSide > 0) robot.moveSide(step * dir);
            else robot.move(step * dir);

            if (useX) positionActual = vu.getX();
            if (useY) positionActual = vu.getY();

            err = positionDesired - positionActual;
            attempts++;

            waitMillis(111);
        }
    }

    private void waitMillis(double millis) {

        sleep((long) millis);
//        millis = Range.clip(millis, 0.01, millis);
//        runtimeWait.reset();
//        while (runtimeWait.nanoseconds() < millis * 1000 * 1000) {
//            idle();
//        }

    }

    private boolean stopTime(double totalSeconds) {
        return totalRuntime.seconds() > totalSeconds;
    }

    // ************************** TELEMETRY HELPER FUNCTIONS *************************************//

    private void updateTelemetry() {

        String field = shortField ? "short" : "long";
        String team = blueTeam ? "blue" : "red";

        telemetry.addData("left drive", "%.0f%%", robot.leftPowerControl * 100);
        telemetry.addData("right drive", "%.0f%%", robot.rightPowerControl * 100);
        telemetry.addData("lift", "%.0f%%", robot.liftControl * 100);
        telemetry.addData("left claw", "%.0f%%", robot.leftClawControl * 100);
        telemetry.addData("right claw", "%.0f%%", robot.rightClawControl * 100);

        telemetry.addData("crr heading", "%.2fdeg", (double) robot.modernRoboticsI2cGyro.getHeading());
        telemetry.addData("set heading", "%.2fdeg", (double) robot.headingControl);
        telemetry.addData("start heading", "%.2fdeg", robot.gameStartHeading);
        telemetry.addData("z angle", "%.2fdeg",
                (double) robot.gyro.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("team", team);
        telemetry.addData("field", field);
        //telemetry.addData("total runtime", "%.0fs", totalRuntime.seconds());

        telemetry.update();
    }

    // ************************** OP END *********************************************************//
}
