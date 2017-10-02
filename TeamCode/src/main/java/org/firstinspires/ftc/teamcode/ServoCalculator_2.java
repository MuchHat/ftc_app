package org.firstinspires.ftc.teamcode;

/**
 * Created by gigela on 10/1/2017.
 */

public class ServoCalculator_2 {

    Angle turretAngle          =  new Angle();
    Angle baseAngle            =  new Angle();
    Angle elbowAngle           =  new Angle();

    Angle wristAngle    =  new Angle();
    Angle clawRightAngle        =  new Angle();
    Angle clawLeftAngle         =  new Angle();

    public void init() {

        turretAngle.Init_45_135(0.140, 0.644, 0.05, 0.897); // turret setup

        baseAngle.Init_45_135(0.25, 0.75, 0.05, 0.95); // TODO
        elbowAngle.Init_45_135(0.404, 0.950, 0.20, 0.95); //elbow setup

        wristAngle.Init_45_135(1.1775, 0.6105, 0.32, 0.89);
        clawRightAngle.Init_45_135(0.221, -0.437, 0.221, 0.55); // right claw setup
        clawLeftAngle.Init_45_135(0.818, 1.582, 0.436, 0.818); // left claw setup
    }
}
