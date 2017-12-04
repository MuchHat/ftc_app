package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//********************************* MAIN OP CLASS ************************************************//

@TeleOp(name = "Auto Blue Short V9", group = "Auto")
// @Disabled
public class Auto_Blue_Short_V9 extends LinearOpMode {

    //********************************* HW VARIABLES *********************************************//
    private Team_Hardware_V9 robot = new Team_Hardware_V9();

    //********************************* MOVE STATES **********************************************//
    private ElapsedTime totalRuntime = null;

    private Run_Glyph glyphRun = new Run_Glyph();
    private Run_Jewel jewelRun = new Run_Jewel();

    // ************************** MAIN LOOP ******************************************************//
    @Override
    public void runOpMode() {

        //********************************* MAIN LOOP INIT ***************************************//
        totalRuntime = new ElapsedTime();

        telemetry.addData("DRIVER", ">>> WAIT WAIT WAIT >>>");
        telemetry.update();

        robot.init(hardwareMap);

        robot.blueTeam = true;
        robot.shortField = true;
        robot.showTeamColor();

        jewelRun.init(robot);
        glyphRun.init(robot, hardwareMap);

        telemetry.addData("DRIVER", ">>>  ADJUST LIFT  >>> PRESS (A) >>>");
        telemetry.update();

        while( !gamepad1.a ){
            liftLoopAuto();
            waitMillis(11);
        }

        telemetry.addData("DRIVER", ">>> PRESS (START) >>>");
        telemetry.update();

        waitForStart();

        totalRuntime.reset();
        robot.imuGyro.start();

        robot.closeClawAuto();
        waitMillis(666);

        while (opModeIsActive()) {

            jewelRun.run();

            robot.showTeamColor();

            glyphRun.run(30 - totalRuntime.seconds());

            robot.stopRobot();
            robot.beaconBlink(2);
            robot.colorBeacon.off();
            stop();
        }
    }

    void liftLoopAuto(){

        robot. liftControl = 0;

        if (Math.abs(gamepad2.right_stick_y) > 0.06) {
            robot.liftControl = -gamepad2.right_stick_y;
        }

        if (Math.abs(gamepad1.right_stick_y) > 0.06) {
            robot.liftControl = -gamepad1.right_stick_y;
        }
        robot.setDrivesByPower();
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
        telemetry.addData("team", team);
        telemetry.addData("field", field);
        telemetry.addData("total runtime", "%.0fs", totalRuntime.seconds());

        telemetry.update();
    }

    // ************************** OP END *********************************************************//
}
