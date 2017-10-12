package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by MuchHat on 2017-10-11.
 */

public class Animator {

    public double rampUp = 666; // distance to get from 0 to 1
    public double rampDown = 888; //distnace to get from 1 to 0
    public double minSpeed = 0.02;
    public double maxSpeed = 0.88;

    double rampUpShape[] = {0.15, 0.33, 0.52, 0.66, 0.78, 0.88, 0.93, 0.97, 1.00, 1.00, 1.00};
    double rampDownShape[] = {1.00, 0.90, 0.80, 0.60, 0.35, 0.25, 0.15, 0.08, 0.05, 0.05, 0.05};
    double shapeSteps = 10;

    double crrPos = 0;
    double nextPos = 0;
    double crrSpeed = 0;
    double nextSpeed = 0;
    double startPos = 0;
    double endPos = 0;
    double crrIteration = 0;
    double maxIterations = 4444; // 4 sec timeout

    boolean useLinear = true;
    boolean useShapes = false;

    double direction = 1.0;
    double error = 0;
    double ratio = 1.0;

    double linearTravel = 1.0;
    double stepTime = 1;

    public void Animator() {

    }

    public void configRamp(double aRampUp, double aRampDown) {

        rampUp = Math.abs( aRampUp );
        rampDown = Math.abs( aRampDown );
    }

    public void configSpeed(double aMinSpeed, double aMaxSpeed, double aLinearTravel, double aStepTime) {

        minSpeed = Math.abs( aMinSpeed );
        maxSpeed = Math.abs( aMaxSpeed );
        linearTravel = Math.abs( aLinearTravel );
        stepTime = Math.abs( aStepTime );
    }

    public void start(double aStartPos, double aEndPos) {

        error = Math.abs(aEndPos - aStartPos);
        direction = aEndPos > aStartPos ? 1.0 : -1.0;
        startPos = aStartPos;
        endPos = aEndPos;

        ratio = 1.0;

        ratio = error / (rampUp + rampDown);
        ratio = Range.clip(ratio, 0, 1);

        crrPos = startPos;
        nextPos = startPos;

        crrSpeed = 0;
        nextSpeed = 0;
    }

    public void modeLinear() {

        useLinear = true;
        useShapes = false;
    }

    public void modeShapes() {

        useLinear = false;
        useShapes = true;
    }

    public double getPos() {

        return nextPos;
    }

    public double getSpeed() {

        return nextSpeed;
    }

    public void advanceStepNoPos() {
        double actualPos = crrPos + stepTime * nextPos * linearTravel;

        advanceStep(actualPos);
    }

    public void advanceStep(double actualPos) {

        error = Math.abs(endPos - actualPos);
        crrPos = actualPos;

        if (error <= 0 || crrIteration > maxIterations) {
            nextPos = endPos;
            nextSpeed = 0;
            return;
        }

        crrIteration++;
        crrSpeed = nextSpeed;
        nextSpeed = maxSpeed;

        if (crrPos <= rampUp) {
            nextSpeed = Math.abs(crrPos / rampUp);
        }
        if (endPos - crrPos <= rampDown) {
            nextSpeed = Math.abs((endPos - crrPos) / rampDown);
        }
        nextSpeed = Range.clip(nextSpeed, minSpeed, maxSpeed);
        nextSpeed = Range.clip(nextSpeed, 0, error / ( stepTime * linearTravel ) );

        nextPos = crrPos + direction * nextSpeed * stepTime * linearTravel;
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
