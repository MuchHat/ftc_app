package org.firstinspires.ftc.teamcode;

/**
 * Created by gigela on 9/27/2017.
 */

public class Arm {

    public double x;
    public double y;
    public double z;

    public Angle  turretAngle          =  new Angle();
    public Angle  baseAngle            =  new Angle();
    public Angle  elbowAngle           =  new Angle();
    public Angle  clawVerticalAngle    =  new Angle();
    public Angle  clawHorizontalAngle  =  new Angle();
    public Angle  clawOpeningAngle     =  new Angle();
    public double clawOpeningMM        = 0;

    Point basePoint  = new Point();
    Point elbowPoint = new Point();
    Point endPoint   = new Point();

    // TODO correct the home positons
    double xZero = 0; // mm
    double yZero = 0; // mm
    double zZero = 0; // mm

    double xHome = 0; // mm
    double yHome = 0; // mm
    double zHome = 0; // mm

    double xFront = 0; // mm
    double yFront = 0; // mm
    double zFront = 0; // mm

    double mmClawOpen  = 222;  // mm
    double mmClawClose = 188; // mm

    double lBase     = 260; // mm
    double lElbow    = 260; // mm
    double lClawArm =  150; // mm
    double lClawGap =  37;  // mm
    // TODO END

    boolean isInitialized = false;

    /* Constructor */
    public void Arm(){

        basePoint.solve_XYZ( 0, 0, 0 );

        // TODO fix the servos tunning
        turretAngle.offsetServo = 0.00; // SERVO 0.2 is 0 PI
        turretAngle.slopeServo  = 1.00; // 1 PI is 0.8 SERVO
        turretAngle.minServo    = 0.05;
        turretAngle.maxServo    = 0.95;

        baseAngle.offsetServo = 0.00; // SERVO 0.2 is 0 PI
        baseAngle.slopeServo  = 1.00; // 1 PI is 0.8 SERVO
        baseAngle.minServo    = 0.05;
        baseAngle.maxServo    = 0.95;

        elbowAngle.offsetServo = 0.00; // SERVO 0.2 is 0 PI
        elbowAngle.slopeServo  = 1.00; // 1 PI is 0.8 SERVO
        elbowAngle.minServo    = 0.05;
        elbowAngle.maxServo    = 0.95;

        clawVerticalAngle.offsetServo = 0.00; // SERVO 0.2 is 0 PI
        clawVerticalAngle.slopeServo  = 1.00; // 1 PI is 0.8 SERVO
        clawVerticalAngle.minServo    = 0.05;
        clawVerticalAngle.maxServo    = 0.95;

        clawHorizontalAngle.offsetServo = 0.00; // SERVO 0.2 is 0 PI
        clawHorizontalAngle.slopeServo  = 1.00; // 1 PI is 0.8 SERVO
        clawHorizontalAngle.minServo    = 0.05;
        clawHorizontalAngle.maxServo    = 0.95;

        clawOpeningAngle.offsetServo = 0.00; // SERVO 0.2 is 0 PI
        clawOpeningAngle.slopeServo  = 1.00; // 1 PI is 0.8 SERVO
        clawOpeningAngle.minServo    = 0.05;
        clawOpeningAngle.maxServo    = 0.95;
        //TODO END
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