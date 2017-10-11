package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by MuchHat on 2017-10-11.
 */

public class Animator {

    public double rampUp = 666; // distance to get from 0 to 1
    public double rampDown = 888; //distnace to get from 1 to 0
    public double minSpeed = 0.03;
    public double maxSpeed = 0.88;

    double currentMaxSpeed = 0.88;
    double currentRampUp = 666;
    double currentRampDown = 888;

    double distance = 0;
    double lastPos = 0;

    ElapsedTime runtime = null;

    double lastSpeed = 0;

    double rampUpShape[] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.0};
    double rampDownShape[] = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.1};
    double shapeSteps = 10;

    public void Animator(double aDistance) {

        runtime = new ElapsedTime();
        runtime.reset();

        distance = aDistance;
        lastPos = 0;

        double ratio = 1.0;

        ratio = Range.clip(distance / (rampUp + rampDown), 0, 1);
        currentMaxSpeed = maxSpeed * ratio;
        currentRampUp = rampUp * ratio;
        currentRampDown = rampDown * ratio;
    }

    public double getSpeed(double currentPos) {

        double newSpeed = lastSpeed;

        if (currentPos < currentRampUp) {
            double ratio = Range.clip(currentPos / rampUp, 0, 1);
            double index = Range.clip(ratio * shapeSteps, 0, shapeSteps - 1);

            newSpeed = maxSpeed * rampUpShape[(int) index];
            lastSpeed = newSpeed;
        }
        if (currentPos > distance - currentRampUp) {
            double ratio = Range.clip((currentRampUp - currentPos) / rampUp, 0, 1);
            double index = Range.clip(ratio * shapeSteps, 0, shapeSteps - 1);

            newSpeed = maxSpeed * rampDownShape[(int) index];
            lastSpeed = newSpeed;
        }

        return Range.clip(newSpeed, minSpeed, maxSpeed);
    }

}

/*
    double velocityByDampedSpring(double targetPos, double currentPos, double currentVelocity, double stepTime) {

        double springConstant = 0.003 / targetPos; //full speed in 15 iterations

        double currentToTarget = targetPos - currentPos;
        double springForce = currentToTarget * springConstant;

        double dampingForce = -currentVelocity * 2 * Math.sqrt(springConstant);
        double force = springForce + dampingForce;

        double newVelocity = currentVelocity + force * stepTime;

        return Range.clip(newVelocity, minVelocity, maxVelocity);
    }
 */
