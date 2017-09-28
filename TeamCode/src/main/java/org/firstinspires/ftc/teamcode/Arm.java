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
    public Angle clawVerticalAngle =  new Angle();
    public Angle clawHorizontalAngle =  new Angle();
    public Angle clawOpeningAngle =  new Angle();
    public double clawOpeningMM = 0;

    Point basePoint = null;
    Point elbowPoint = null;
    Point endPoint = null;

    double xZero = 0;
    double yZero = 0;
    double zZero = 0;

    double xHome = 0;
    double yHome = 0;
    double zHome = 0;

    double xFront = 0;
    double yFront = 0;
    double zFront = 0;

    double mmClawOpen = 160;
    double mmClawClose = 55;

    double lBase = 222;
    double lElbow = 222;
    double lClawArm = 11;
    double lClawGap = 11;

    boolean isInitialized = false;

    /* Constructor */
    public void Arm(){

        basePoint = new Point();
        elbowPoint = new Point();
        endPoint = new Point();

        basePoint.solve_XYZ( 0, 0, 0 );
    }

    public void copyFrom( Arm anotherArm ){
        solve_XYZ( anotherArm.x, anotherArm.y, anotherArm.z  );
    }

    public void solve_Claw( double mm ){

        Triangle clawTriangle = new Triangle();

        clawTriangle.solve_SSS( ( mm - lClawGap ) / 2, lClawArm, Math.sqrt( ( ( mm - lClawGap ) / 2 ) * ( ( mm - lClawGap ) / 2 ) + lClawArm * lClawArm ) );
        clawOpeningAngle.solve_AnglePI( clawTriangle.a1 );

        clawOpeningMM = mm;
    }

    public void solve_Servos( double ts, double bs, double es ){

        turretAngle.solve_AngleServo( ts );
        baseAngle.solve_AngleServo( bs );
        elbowAngle.solve_AngleServo( es );

        elbowPoint.solve_R_AZ_AX( lBase, baseAngle.anglePI, turretAngle.anglePI );
        endPoint.solve_R_AZ_AX( lElbow, elbowAngle.anglePI - Math.PI / 2, elbowPoint.ax );

        x = endPoint.x;
        y = endPoint.y;
        z = endPoint.z;

        clawVerticalAngle.solve_AnglePI( Math.PI / 2 + elbowPoint.az );
        clawHorizontalAngle.solve_AnglePI( - turretAngle. anglePI );

        isInitialized = true;
    }

    public void solve_XYZ( double x, double y, double z ){

        endPoint.solve_XYZ( x, y, z );

        Triangle elbowTriangle = new Triangle();

        elbowTriangle.solve_SSS( lBase, lElbow, endPoint.r );

        elbowPoint.solve_R_AZ_AX( lBase, endPoint.az + elbowTriangle.a2, endPoint.ax );

        x = endPoint.x;
        y = endPoint.y;
        z = endPoint.z;

        turretAngle.solve_AnglePI( elbowPoint.ax );
        elbowAngle.solve_AnglePI( elbowTriangle.a3 );

        clawVerticalAngle.solve_AnglePI( Math.PI / 2 + elbowPoint.az );
        clawHorizontalAngle.solve_AnglePI( - turretAngle. anglePI );

        isInitialized = true;
    }
}