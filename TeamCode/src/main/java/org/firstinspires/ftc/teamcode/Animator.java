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

    double actualMaxSpeed = 0.88;
    double actualRampUp = 666;
    double actualRampDown = 888;

    double distance = 0;

    double lastSpeed = 0;

    double rampUpShape[] = {0.15, 0.33, 0.52, 0.66, 0.78, 0.88, 0.93, 0.97, 1.00, 1.00, 1.00};
    double rampDownShape[] = {1.00, 0.90, 0.80, 0.60, 0.35, 0.25, 0.15, 0.08, 0.05, 0.05, 0.05};
    double shapeSteps = 10;

    public void Animator() {

    }

    public void init(double aDistance) {

        distance = Math.abs(aDistance);
        double ratio = 1.0;

        ratio = distance / (rampUp + rampDown);
        ratio = Range.clip(ratio, 0, 1);

        actualMaxSpeed = maxSpeed * ratio;
        actualRampUp = rampUp * ratio;
        actualRampDown = rampDown * ratio;
    }

    public double getSpeed(double currentPos) {

        currentPos = Math.abs(currentPos);

        if (currentPos >= distance || currentPos <=3 ) {
            lastSpeed = minSpeed;
            return minSpeed;
        }
        double newSpeed = lastSpeed;
        double ratio = 1.0;

        if (currentPos <= actualRampUp) {
            ratio = currentPos / rampUp;
        }
        if (distance - currentPos <= actualRampDown) {
            ratio = (distance - currentPos) / rampDown;
        }

        newSpeed = maxSpeed * ratio;
        lastSpeed = newSpeed;

        return Range.clip(newSpeed, minSpeed, actualMaxSpeed);
    }

    public double getSpeedShape(double currentPos) {

        currentPos = Math.abs(currentPos);

        if (currentPos >= distance || currentPos == 0) {
            lastSpeed = minSpeed;
            return minSpeed;
        }
        double newSpeed = lastSpeed;

        if (currentPos <= actualRampUp) {
            double ratio = Range.clip(currentPos / rampUp, 0, 1);
            double index = Range.clip(ratio * shapeSteps, 0, shapeSteps - 1);

            newSpeed = Math.max(maxSpeed * rampUpShape[(int) index], lastSpeed);
            lastSpeed = newSpeed;
        }
        if (distance - currentPos <= actualRampDown) {
            double ratio = Range.clip((distance - currentPos) / rampDown, 0, 1);
            double index = shapeSteps - 1 - Range.clip(ratio * shapeSteps, 0, shapeSteps - 1);

            newSpeed = Math.min(maxSpeed * rampDownShape[(int) index], lastSpeed);
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
