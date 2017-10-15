package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// ************************** ANIMATOR HELPER CLASS **********************************************//

public class Animator {

    // ************************** VARIABLES *******************************************************//

    public double rampUpAbs = 666; // distance to get from 0 to 1
    public double rampDownAbs = 888; //distnace to get from 1 to 0
    public double minSpeedAbs = 0.02;
    public double maxSpeedAbs = 0.88;

    double nextPos = 0;
    double crrSpeedAbs = 0;
    double nextSpeedAbs = 0;
    double startPos = 0;
    double endPos = 0;
    double crrPos = 0;
    double crrIteration = 0;
    double maxIterations = 6666; // 6 sec timeout
    double distanceAbs = 0;

    double direction = 1.0;
    double errorAbs = 0;
    double toleranceAbs = 0;
    double ratioAbs = 1.0;

    double linearTravelAbs = 1.0;
    double stepTimeAbs = 1;

    ElapsedTime animatorRuntime = null;

    public void Animator() {

        animatorRuntime = new ElapsedTime();
    }

    // ************************** INIT AND CONFIG  ***********************************************//

    public void configRamp(double aRampUp, double aRampDown) {

        rampUpAbs = Math.abs(aRampUp);
        rampDownAbs = Math.abs(aRampDown);
        toleranceAbs = rampUpAbs / 10;
    }

    public void configSpeed(double aMinSpeed, double aMaxSpeed, double aLinearTravel, double aStepTime) {

        minSpeedAbs = Math.abs(aMinSpeed);
        maxSpeedAbs = Math.abs(aMaxSpeed);
        linearTravelAbs = Math.abs(aLinearTravel);
        stepTimeAbs = Math.abs(aStepTime);
    }

    // ************************** EXECUTION  *****************************************************//

    public void start(double aStartPos, double aEndPos) {

        distanceAbs = Math.abs(aEndPos - aStartPos);
        toleranceAbs = distanceAbs * 0.05;
        errorAbs = distanceAbs;
        direction = aEndPos > aStartPos ? 1.0 : -1.0;
        startPos = aStartPos;
        endPos = aEndPos;

        ratioAbs = 1.0;

        ratioAbs = errorAbs / (rampUpAbs + rampDownAbs);
        ratioAbs = Range.clip(ratioAbs, 0.001, 1);

        nextPos = startPos;
        crrPos = startPos;

        crrSpeedAbs = 0;
        nextSpeedAbs = 0;

        animatorRuntime.reset();
    }

    public double getPos() {

        animatorRuntime.reset();
        return nextPos;
    }

    public double getSpeed() {

        animatorRuntime.reset();
        return nextSpeedAbs;
    }

    public void advanceStepNoPos() {

        double actualStepTime = animatorRuntime.nanoseconds() / (1000 * 1000);
        double actualPos = crrPos + direction * nextSpeedAbs * actualStepTime * linearTravelAbs;
        animatorRuntime.reset();
        advanceStep(actualPos);
    }

    public void advanceStep(double actualPos) {

        animatorRuntime.reset();

        if (direction >= 0) actualPos = Range.clip(actualPos, startPos, endPos);
        if (direction < 0) actualPos = Range.clip(actualPos, endPos, startPos);

        errorAbs = Math.abs(endPos - actualPos);
        distanceAbs = Math.abs(endPos - startPos) - errorAbs;

        if (errorAbs <= toleranceAbs) {
            nextPos = endPos;
            nextSpeedAbs = 0;
            errorAbs = 0;
            distanceAbs = 0;
            return;
        }

        crrIteration++;
        crrSpeedAbs = nextSpeedAbs;
        nextSpeedAbs = maxSpeedAbs * ratioAbs;

        if (distanceAbs <= rampUpAbs) {
            double rampPos = (distanceAbs / rampUpAbs) / ratioAbs;
            nextSpeedAbs = getS(rampPos) * ratioAbs * maxSpeedAbs;
        }
        if (errorAbs <= rampDownAbs) {
            double rampPos = (errorAbs / rampDownAbs) / ratioAbs;
            nextSpeedAbs = getS(rampPos) * ratioAbs * maxSpeedAbs;
        }
        nextSpeedAbs = Range.clip(nextSpeedAbs, minSpeedAbs, maxSpeedAbs);
        nextSpeedAbs = Range.clip(nextSpeedAbs, 0, errorAbs / (stepTimeAbs * linearTravelAbs));

        crrPos = actualPos;
        nextPos = crrPos + direction * nextSpeedAbs * stepTimeAbs * linearTravelAbs;
    }

    double getS(double ratio) {

        double jerk = 1.355;
        double div = ((1 - Math.cos(Math.pow(0.5, jerk) * Math.PI)) / 2) / 2;
        boolean useS = true;

        double s = ratio;

        ratio = Range.clip(ratio, 0, 1);
        if (useS && ratio <= 0.5) {
            s = ((1 - Math.cos(Math.pow(ratio, jerk) * Math.PI)) / 2) / div / 2;
            s = Range.clip(s, 0, 0.5);
        } else if (useS && ratio > 0.5) {
            s = 1 - (((1 - Math.cos(Math.pow(1 - ratio, jerk) * Math.PI)) / 2) / div / 2);
            s = Range.clip(s, 0.5, 1);
        }

        return Range.clip(s, 0, 1);
    }

    // ************************** END CLASS  *****************************************************//
}

// ************************** OLD ****************************************************************//
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

// ************************** END OLD ************************************************************//
