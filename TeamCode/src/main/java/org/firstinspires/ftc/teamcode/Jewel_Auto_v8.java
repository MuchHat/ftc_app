package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/* NOTES

https://github.com/ftctechnh/ftc_app/wiki/Identifying-Vuforia-VuMarks

Vuforia key

Ac6cr63/////AAAAGUsQTEyyG0kggwF13U8WoMlPgXZiUoKR9pf2nlfhVVfvDXFsTn0wufoywxzibq+y5BGBa2cChKWAcUkKaD9/ak5lwCm9Wp3Osk9omsMR0YYoxt4TuPktrflK4HuTH8cOAQA8YDuOs/SO/cgOmWbQZtRXN/lFkUwGZA9eiV5D8730BG2SBLPR4A9rcFs0Fp/yPgcm4Zsh5Kv2Ct8XjJXmXk5mAjERZ5B6hKQzf/4wd9tSQ6BeQLvsgd5nI0Pj+K1NHI4EyHdFyxCPu91AMcCsXCLjkABfYt11Zhxu1uYaFF/AcN3eBHRwprVpDEBBXOMnD4BRCj0xxYYPWWO6g4gcjqBPgBos5nCDk43KipEeX22z

 */

//********************************* MAIN OP CLASS ************************************************//

@TeleOp(name = "Jewel Auto V8", group = "Team")
// @Disabled
public class Jewel_Auto_v8 extends LinearOpMode {

    boolean foundBlue = false;
    boolean foundRed = false;
    int foundPos = 0;
    //********************************* HW VARIABLES *********************************************//
    private Team_Hardware_V3 robot = new Team_Hardware_V3();
    //********************************* MOVE STATES **********************************************//
    private ElapsedTime totalRuntime = null;

    //********************************* CONSTANTS ************************************************//
    private Boolean blueTeam = true;
    private Boolean rightField = true;
    private Boolean loaded = false;

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

        while (opModeIsActive() && loaded) {

            //********************************* CONTROL LOOP *************************************//

            double crrLoopTime = loopRuntime.nanoseconds() / 1000000; // covert to millis
            loopRuntime.reset();

            updateTelemetry();

            robot.colorBeacon.yellow();

            // first find the right color jewel

            double armFindJewelB[] = {0.66, 0.70, 0.70, 0.70};
            double armFindJewelE[] = {0.15, 0.30, 0.40, 0.45};

            double armKnockB[] = {0.68, 0.65, 0.65, 0.60};
            double armKnockE[] = {0.30, 0.30, 0.40, 0.40};

            double armExtendedB = 0.68;
            double armExtendedE = 0.30;

            int findPositions = 4;
            robot.moveArm(armExtendedB, armExtendedE);
            waitMillis(555);

            for (int i = 0; i < findPositions; i++) {

                double crrBase = armFindJewelB[i];
                double crrElbow = armFindJewelE[i];

                robot.moveArm(crrBase, crrElbow);
                waitMillis(111);

                if (foundJewel()) {
                    foundPos = i;
                    robot.moveArm(armExtendedB, armExtendedE);
                    waitMillis(555);
                    break;
                }
            }
            for (int i = 0; i < 3; i++) {

                waitMillis(111);
                robot.colorBeacon.off();
                waitMillis(111);
                robot.colorBeacon.yellow();
            }

            // move the jewel
            if (foundBlue || foundRed) {

                //TODO add logic to account for fw/rev and team color

                robot.move(11);
                waitMillis(222);

                double crrBase = armKnockB[foundPos];
                double crrElbow = armKnockE[foundPos];
                robot.moveArm(crrBase, crrElbow);
                waitMillis(222);

                robot.move(-11);
                waitMillis(222);

                robot.move(11);
                waitMillis(222);

                robot.moveArm(robot.armPosZero[0], robot.armPosZero[1]);
                waitMillis(222);

                robot.colorBeacon.green();

            } else if (!foundBlue || !foundRed) {
                robot.colorBeacon.yellow();
            }

            robot.stopRobot();
            stop(); //stop the opMode

            //********************************* END LOOP *****************************************//
        }
    }

    boolean foundJewel() {

        if (robot.colorSensor.red() > robot.colorSensor.blue()) {
            foundRed = true;
            foundBlue = false;
            robot.colorBeacon.red();
            return true;
        }
        if (robot.colorSensor.blue() > robot.colorSensor.red()) {
            foundBlue = true;
            foundRed = false;
            robot.colorBeacon.blue();
            return true;
        }

        return false;
    }


    // ************************** GENERAL MOVE HELPER FUNCTIONS  *********************************//

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

        String field = rightField ? "right" : "left";
        String team = blueTeam ? "blue" : "red";

        telemetry.addData("left drive", "%.0f%%", robot.leftDriveControl * 100);
        telemetry.addData("right drive", "%.0f%%", robot.rightDriveControl * 100);
        telemetry.addData("base", "%.0f%%", robot.baseControl * 100);
        telemetry.addData("elbow", "%.0f%%", robot.elbowControl * 100);
        telemetry.addData("lift", "%.0f%%", robot.liftControl * 100);
        telemetry.addData("left claw", "%.0f%%", robot.leftClawControl * 100);
        telemetry.addData("right claw", "%.0f%%", robot.rightClawControl * 100);

        telemetry.addData("crr heading", "%.2fdeg", (double) robot.modernRoboticsI2cGyro.getHeading());
        telemetry.addData("set heading", "%.2fdeg", (double) robot.headingControl);
        telemetry.addData("start heading", "%.2fdeg", robot.gameStartHeading);
        telemetry.addData("z angle", "%.2fdeg",
                (double) robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("team", team);
        telemetry.addData("field", field);
        //telemetry.addData("total runtime", "%.0fs", totalRuntime.seconds());

        telemetry.update();
    }

    // ************************** OP END *********************************************************//
}

