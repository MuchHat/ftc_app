package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@TeleOp(name = "Auto Blue Long V9", group = "Competition")
//@Disabled
public class Auto_Blue_Long_V9 extends LinearOpMode {

    //********************************* HW VARIABLES *********************************************//
    private Team_Hardware_V9 robot = new Team_Hardware_V9();

    //********************************* MOVE STATES **********************************************//
    private ElapsedTime totalRuntime = null;
    private Boolean loaded = false;

    private Run_Glyph glyphRun = new Run_Glyph();
    private Run_Jewel jewelRun = new Run_Jewel();

    // ************************** MAIN LOOP ******************************************************//
    @Override
    public void runOpMode() {

        //********************************* MAIN LOOP INIT ***************************************//
        robot.init(hardwareMap);

        ElapsedTime controlRuntime = new ElapsedTime();
        ElapsedTime loopRuntime = new ElapsedTime();
        totalRuntime = new ElapsedTime();

        robot.blueTeam = false;
        robot.shortField = true;
        robot.showTeamColor();

        telemetry.addData("starting vuforia", "... do NOT move");
        telemetry.update();

        jewelRun.init(robot);
        glyphRun.init(robot, hardwareMap);

        robot.modernRoboticsI2cGyro.calibrate();
        controlRuntime.reset();        // Wait until the gyro calibration is complete
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
        robot.autoTimer.reset();

        while (opModeIsActive() && loaded) {

            jewelRun.run();

            robot.showTeamColor();

            glyphRun.run();

            robot.beaconBlink(3);
            robot.colorBeacon.off();

            robot.stopRobot();
            stop();

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

        String field = robot.shortField ? "short" : "long";
        String team = robot.blueTeam ? "blue" : "red";

        telemetry.addData("left drive", "%.0f%%", robot.leftPowerControl * 100);
        telemetry.addData("right drive", "%.0f%%", robot.rightPowerControl * 100);
        telemetry.addData("lift", "%.0f%%", robot.liftControl * 100);
        telemetry.addData("left claw", "%.0f%%", robot.leftClawControl * 100);
        telemetry.addData("right claw", "%.0f%%", robot.rightClawControl * 100);
        //telemetry.addData("Vuforia X: ", ".0f%%", glyphRun.vu.getX());
        //telemetry.addData("Vuforia Y: ", ".0f%%", glyphRun.vu.getY());

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
