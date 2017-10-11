package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by MuchHat on 2017-10-11.
 */

public class animator {

    double distance = 0;
    double lastPos = 0;

    public double rampUp = 666; // distance to get from 0 to 1
    public double rampDown = 888; //distnace to get from 1 to 0

    public double minSpeed = 0.03;
    public double maxSpeed = 0.88;

    ElapsedTime runtime = null;

    public void animator(double aDistance) {

        runtime = new ElapsedTime();
        runtime.reset();

        distance = aDistance;
        lastPos = 0;
    }

    public double getSpeed(double currentPos) {
        return 0;
    }

}

