package org.firstinspires.ftc.teamcode;

/**
 * Created by gigela on 9/27/2017.
 */

public class Arm {

    public double x;
    public double y;
    public double z;

    public double turretAngle = 0;
    public double baseAngle = 0;
    public double elbowAngle = 0;
    public double wristAngle = 0;
    public double clawLeftAngle = 0;
    public double clawRightAngle = 0;

    Point base = null;
    Point elbow = null;
    Point wrist = null;
    Point claw = null;

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

    public void solve_anglesTBE( double turretAngle, double baseAngle, double elbowAngle ){

        elbow.solve_R_AZ_AX( lBase, baseAngle, turretAngle );
        wrist.solve_R_AZ_AX( lElbow, elbowAngle - Math.PI / 2, elbow.ax );

        x = wrist.x;
        y = wrist.y;
        z = wrist.z;

        claw.solve_XYZ( x, y + lClaw, z );

        turretAngle = elbow.ax;
        elbowAngle = Math.PI / 2 + elbow.az;
        wristAngle = Math.PI - elbow.az;

        isValid = true;
    }

    /*************** BELOW ARE NOT COMPLETE ***************/

    public void solve_XYZ( double x, double y, double z ){

        isValid = true;
    }
}