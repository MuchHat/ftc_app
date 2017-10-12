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

    double rampUpShape[] = {0.15, 0.33, 0.52, 0.66, 0.78, 0.88, 0.93, 0.97, 1.00, 1.00, 1.00};
    double rampDownShape[] = {1.00, 0.90, 0.80, 0.60, 0.35, 0.25, 0.15, 0.08, 0.05, 0.05, 0.05};
    double shapeSteps = 10;

    double nextPos = 0;
    double crrSpeedAbs = 0;
    double nextSpeedAbs = 0;
    double startPos = 0;
    double endPos = 0;
    double crrPos = 0;
    double crrIteration = 0;
    double maxIterations = 6666; // 6 sec timeout
    double distanceAbs = 0;

    boolean useLinear = true;
    boolean useShapes = false;

    double direction = 1.0;
    double errorAbs = 0;
    double prevErrorAbs = 0;
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

        errorAbs = Math.abs(aEndPos - aStartPos);
        prevErrorAbs = errorAbs;
        distanceAbs = errorAbs;
        direction = aEndPos > aStartPos ? 1.0 : -1.0;
        startPos = aStartPos;
        endPos = aEndPos;
        toleranceAbs = Math.min(toleranceAbs, distanceAbs / 10);

        ratioAbs = 1.0;

        ratioAbs = errorAbs / (rampUpAbs + rampDownAbs);
        ratioAbs = Range.clip(ratioAbs, 0, 1);

        nextPos = startPos;
        crrPos = startPos;

        crrSpeedAbs = 0;
        nextSpeedAbs = 0;

        animatorRuntime.reset();
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

        if ((direction > 0 && actualPos >= endPos) ||
                (direction < 0 && actualPos <= endPos) ||
                (crrIteration > maxIterations) ||
                (errorAbs < toleranceAbs) ||
                ((errorAbs > prevErrorAbs + toleranceAbs) && (errorAbs < 2 * toleranceAbs))) {
            nextPos = endPos;
            nextSpeedAbs = 0;
            prevErrorAbs = errorAbs;
            errorAbs = 0;
            distanceAbs = 0;
            return;
        }

        crrIteration++;
        crrSpeedAbs = nextSpeedAbs;
        nextSpeedAbs = maxSpeedAbs;
        prevErrorAbs = errorAbs;

        errorAbs = Math.abs(endPos - actualPos);
        distanceAbs = Math.abs(distanceAbs - errorAbs);

        if (distanceAbs <= rampUpAbs) {
            nextSpeedAbs = getS( Math.abs(distanceAbs / rampUpAbs) );
        }
        if (errorAbs <= rampDownAbs) {
            nextSpeedAbs = getS( Math.abs(rampDownAbs - errorAbs) / rampDownAbs );
        }
        nextSpeedAbs = Range.clip(nextSpeedAbs, minSpeedAbs, maxSpeedAbs);
        nextSpeedAbs = Range.clip(nextSpeedAbs, 0, errorAbs / (stepTimeAbs * linearTravelAbs));

        crrPos = actualPos;
        nextPos = crrPos + direction * nextSpeedAbs * stepTimeAbs * linearTravelAbs;
    }

    double getS( double ratio ){


        return ratio;
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
