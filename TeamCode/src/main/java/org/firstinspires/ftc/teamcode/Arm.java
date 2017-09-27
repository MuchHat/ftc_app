package org.firstinspires.ftc.teamcode;

/**
 * Created by gigela on 9/27/2017.
 */

public class Arm {

    public double x;
    public double y;
    public double z;

    public Angle turretAngle = new Angle();
    public Angle baseAngle =  new Angle();
    public Angle elbowAngle =  new Angle();
    public Angle wristAngle =  new Angle();

    // TODO on the claw
    public Angle clawAngle =  new Angle(); // those are mirror and controlled by one channel

    Point basePoint = null;
    Point elbowPoint = null;
    Point wristPoint = null;
    Point clawPoint = null;

    double lBase = 222;
    double lElbow = 222;
    double lClaw = 11;

    boolean isInitialized = false;

    /* Constructor */
    public void Arm(){

        basePoint = new Point();
        elbowPoint = new Point();
        wristPoint = new Point();
        clawPoint = new Point();

        basePoint.solve_XYZ( 0, 0, 0 );
    }

    public void solve_Servos( double ts, double bs, double es ){

        turretAngle.solve_AngleServo( ts );
        baseAngle.solve_AngleServo( bs );
        elbowAngle.solve_AngleServo( es );

        elbowPoint.solve_R_AZ_AX( lBase, baseAngle.anglePI, turretAngle.anglePI );
        wristPoint.solve_R_AZ_AX( lElbow, elbowAngle.anglePI - Math.PI / 2, elbowPoint.ax );

        x = wristPoint.x;
        y = wristPoint.y;
        z = wristPoint.z;

        clawPoint.solve_XYZ( x, y + lClaw, z );

        isInitialized = true;
    }

    public void solve_XYZ( double x, double y, double z ){

        wristPoint.solve_XYZ( x, y, z );

        Triangle elbowTriangle = new Triangle();

        elbowTriangle.solve_SSS( lBase, lElbow, wristPoint.r );

        elbowPoint.solve_R_AZ_AX( lBase, wristPoint.az + elbowTriangle.a2, wristPoint.ax );

        x = wristPoint.x;
        y = wristPoint.y;
        z = wristPoint.z;

        clawPoint.solve_XYZ( x, y + lClaw, z );

        turretAngle.solve_AnglePI( elbowPoint.ax );
        elbowAngle.solve_AnglePI( Math.PI / 2 + elbowPoint.az );
        wristAngle.solve_AnglePI( Math.PI - elbowPoint.az );

        isInitialized = true;
    }
}