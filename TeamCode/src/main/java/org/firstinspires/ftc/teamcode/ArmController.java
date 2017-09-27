package org.firstinspires.ftc.teamcode;

/**
 * Created by gigela on 9/27/2017.
 */

public class ArmController {

    Arm origin = null;
    Arm current = null;
    Arm next = null;
    Arm destination = null;

    Point clawOrigin = null;
    Point clawDestination = null;

    double currentSpeed_mms = 0;
    double maxSpeed_mms = 0;
    double rampAccel_mmsmm = 0;

    double platformHeight = 222;
    double baseToEdgeX = 66;
    double baseToEdgeY = 88;

    boolean isInitialized = false;

    public void moveIncremental( double ix, double iy, double iz ){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop
    }

    public void moveToPosition( double ax, double ay, double az ){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop

        isInitialized = true;
    }

    public void moveToPositionZero(){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop

        isInitialized = true;
    }

    public void moveToPositionHome(){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop

        isInitialized = true;
    }

    public void moveToPositionFront(){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop

        isInitialized = true;
    }
    public void check(){

        // adjust for not bumping in the chasis
    }
}
