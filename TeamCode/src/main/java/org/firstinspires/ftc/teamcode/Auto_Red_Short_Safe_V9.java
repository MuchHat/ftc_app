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

@TeleOp(name = "Auto Red Short Safe V9", group = "Competition")
// @Disabled
public class Auto_Red_Short_Safe_V9 extends LinearOpMode {

    //********************************* HW VARIABLES *********************************************//
    private Team_Hardware_V9 robot = new Team_Hardware_V9();

    //********************************* MOVE STATES **********************************************//
    private ElapsedTime totalRuntime = null;

    private Boolean blueTeam = false;
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
        else robot.colorBeacon.red();

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

        vu.init(hardwareMap);

        robot.rightClaw.setPosition(0.59);
        robot.leftClaw.setPosition(0.54);

        waitForStart();

        while (opModeIsActive() && loaded) {


            double crrLoopTime = loopRuntime.nanoseconds() / 1000000; // covert to millis
            loopRuntime.reset();
            updateTelemetry();

            //********************************* AUTO MOVE TO THE SAFE ZONE ***********************//

            // get using encoders in the general area
            // Right x: 356 Middle: 421 3rd: 477

            int[] moveDistance = {580, 358, 161};

            robot.move(178);
            if(vu.targetSeen())
                robot.colorBeacon.green();
            waitMillis(1000);
            robot.move(432);
            int index = 0;
            if (vu.targetSeen())
            {
                robot.colorBeacon.green();
                index = vu.lastTargetSeenNo;
            }
            else {robot.colorBeacon.yellow();}
            waitMillis(111);
            robot.move(-260);
            if (vu.targetSeen()) {robot.colorBeacon.green();}
            else {robot.colorBeacon.yellow();}
            waitMillis(1000);
            if (index != 0) {robot.move(moveDistance[index-1]);}
            else {robot.colorBeacon.red();}
            waitMillis(333);

            if (vu.targetSeen()) {
                robot.colorBeacon.green();

                int errDis = 20; // 1 left 2 center 3 right
                int[] xValues = {460, 415, 340}; // Desired value for left, right, and middle
                index = vu.getLastTargetSeenNo() - 1;
                int desiredX = xValues[index];

                waitMillis(333);
                robot.move(50);

                double vuX = vu.getX();
                double attempts = 0;

                /*while (vuX < desiredX && attempts < 33) {
                    robot.colorBeacon.purple();
                    vuX = vu.getX();
                    if (desiredX - vuX < 100) {
                        robot.move(10);
                        waitMillis(33);

                    } else {
                        robot.move(20);
                        waitMillis(33);

                    }
                    attempts++;
                }
                if (blueTeam) robot.colorBeacon.blue();
                else robot.colorBeacon.red();*/

                 while(vu.getX() - desiredX > errDis) {
                    robot.colorBeacon.purple();
                    robot.move(-10);
                }
                if(blueTeam)
                    robot.colorBeacon.blue();
                 else
                     robot.colorBeacon.red();
            }
            //turn to put the glyph in

            robot.colorBeacon.teal();
            robot.turn(85);
            robot.colorBeacon.white();
            updateTelemetry();
            waitMillis(555);

            robot.stopRobot();
            updateTelemetry();
            waitMillis(555);

            //put the glyph in
            for( int halfInches = 0; halfInches < 11; halfInches++){
                robot.move(0.5 * 25.4);
                waitMillis(33);
            }

            robot.rightClaw.setPosition(0.75);
            robot.leftClaw.setPosition(0.22);
            waitMillis(111);

            robot.move(-100);
            for( int halfInches = 0; halfInches < 6; halfInches++){
                robot.move(0.5 * 25.4);
                waitMillis(33);
            }

            robot.stopRobot();

            stop(); //stop the opMode

            //********************************* END LOOP *****************************************//
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
        telemetry.addData("Vuforia X: ", ".0f%%", vu.getX());
        telemetry.addData("Vuforia Y: ", ".0f%%", vu.getY());

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
