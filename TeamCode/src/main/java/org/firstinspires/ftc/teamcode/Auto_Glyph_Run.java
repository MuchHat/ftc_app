package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static android.os.SystemClock.sleep;

//********************************* MAIN OP CLASS ************************************************//

public class Auto_Glyph_Run {

    private Vu vu = new Vu();
    private HardwareMap hwMap = null;

    //********************************* HW VARIABLES *********************************************//
    Team_Hardware_V9 robot = null;

    //********************************* CONSTANTS ************************************************//
    Boolean blueTeam = true;
    Boolean shortField = true;

    void init(Team_Hardware_V9 aRobot, HardwareMap aHwMap, boolean aBlueTeam, boolean aShortField) {

        hwMap = aHwMap;
        robot = aRobot;

        blueTeam = aBlueTeam;
        shortField = aShortField;

        robot.rightClaw.setPosition(0.59);
        robot.leftClaw.setPosition(0.54);

        vu.init(hwMap);

    }

    void runAuto() {

        if (robot == null) {
            return;
        }

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
        robot.turnTo3();
        robot.colorBeacon.white();
        waitMillis(555);

        robot.stopRobot();
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

        //********************************* END LOOP *****************************************//
    }

    private void waitMillis(double millis) {

        sleep((long) millis);
//        millis = Range.clip(millis, 0.01, millis);
//        runtimeWait.reset();
//        while (runtimeWait.nanoseconds() < millis * 1000 * 1000) {
//            idle();
//        }

    }
 }

