package org.firstinspires.ftc.teamcode;

/**
 * Created by gigela on 9/27/2017.
 */

public class Arm {

    public double x;
    public double y;
    public double z;

    Point base = null;
    Point elbow = null;
    Point wrist = null;
    Point claw = null;

    Point wristOrigin = null;
    Point wristDestination = null;

    Point clawOrigin = null;
    Point clawDestination = null;

    double currentSpeed_mms = 0;
    double maxSpeed_mms = 0;
    double rampAccel_mmsmm = 0;

    double lBase = 222;
    double lElbow = 222;
    double lClaw = 11;

    boolean isValid = false;

    /* Constructor */
    public void Arm(){

        base = new Point();
        elbow = new Point();
        wrist = new Point();
        claw = new Point();

        base.solve_XYZ( 0, 0, 0 );
    }

    public void solve_TBE( double turretAngle, double baseAngle, double elbowAngle ){

        elbow.solve_R_AZ_AX( lBase, baseAngle, turretAngle );
        wrist.solve_R_AZ_AX( lElbow, elbowAngle - Math.PI / 2, elbow.ax );

        x = wrist.x;
        y = wrist.y;
        z = wrist.z;

        claw.solve_XYZ( x, y + lClaw, z );

        isValid = true;
    }

    /*************** BELOW ARE NOT COMPLETE ***************/

    public void solve_XYZ( double x, double y, double z ){

        isValid = true;
    }

    public void moveIncremental( double ix, double iy, double iz ){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop
    }

    public void moveToPosition( double ax, double ay, double az ){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop
    }

    public void moveToPositionZero(){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop
    }

    public void moveToPositionHome(){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop
    }

    public void moveToPositionFront(){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop
    }

    public void staySafe(){

        // adjust for not bumping in the chasis
    }
}