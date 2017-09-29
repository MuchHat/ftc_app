package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Created by gigela on 9/27/2017.
 */

public class Arm {

    private double x = 0;
    private double y = 0;
    private double z = 0;

    public double t_pi = 0;
    public double b_pi = 0;
    public double e_pi = 0;

    private double teta = 0;
    private double phi = 0;

    private double r = 0;

    public Angle  turretAngle          =  new Angle();
    public Angle  baseAngle            =  new Angle();
    public Angle  elbowAngle           =  new Angle();
    public Angle  clawVerticalAngle    =  new Angle();
    public Angle  clawHorizontalAngle  =  new Angle();
    public Angle  clawOpeningAngle     =  new Angle();
    public double clawOpeningMM        = 0;

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

    double xMax = 450; // mm
    double xMin = 45; // mm
    double yMax = 450; // mm
    double yMin = 0; // mm
    double zMax = 450; // mm
    double zMin = -150; // mm
    double rMax = 450; // mm

    // TODO END

    boolean isInitialized = false;
    String log = new String();

    /* Constructor */
    public void Arm(){

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

        solve_Claw( lClawGap );
    }

    public double getX(){

    }
    public double getY(){

    }
    public double getZ(){

    }
    public double getBaseServo(){

    }
    public double getElbowServo(){

    }
    public double getWristHorizontalServo(){

    }
    public double getWristVerticalServo(){

    }

    public void copyFrom( Arm anotherArm ){
        solve_XYZ( anotherArm.x, anotherArm.y, anotherArm.z  );
        solve_Claw( anotherArm.clawOpeningMM );
    }

    public void solve_Claw( double mm ){

        Triangle clawTriangle = new Triangle();

        mm = Math.abs( mm );

        clawTriangle.solve_SSS( ( mm - lClawGap ) / 2, lClawArm,
                Math.sqrt( ( ( mm - lClawGap ) / 2 ) * ( ( mm - lClawGap ) / 2 ) +
                        lClawArm * lClawArm ) );
        clawOpeningAngle.solve_anglePI( clawTriangle.a1 );

        clawOpeningMM = mm;
    }

    public double distanceTo( Arm anotherArm ){

        return Math.sqrt( ( x - anotherArm.x ) * ( x - anotherArm.x ) +
                ( y - anotherArm.y ) * ( y - anotherArm.y ) +
                ( z - anotherArm.z ) * ( z - anotherArm.z ) );

    }

    public void solve_Servos( double ts, double bs, double es ) {

        // use polar coordinates
        // all angles are 0 to PI
        // https://en.wikipedia.org/wiki/Spherical_coordinate_system

        turretAngle.solve_angleServo(ts);
        baseAngle.solve_angleServo(bs);
        elbowAngle.solve_angleServo(es);

        t_pi = turretAngle.anglePI;
        b_pi = baseAngle.anglePI;
        e_pi = elbowAngle.anglePI;

        teta = Math.PI - t_pi;
        phi = b_pi - Math.PI / 2;

        Triangle elbowTriangle = new Triangle();
        elbowTriangle.solve_SSA(lBase, lElbow, e_pi);

        r = elbowTriangle.l3;
        if (r > rMax) r = rMax;

        x = r * Math.sin(teta) * Math.cos(phi);
        y = r * Math.sin(teta) * Math.sin(phi);
        z = r * Math.cos(teta);

        isInitialized = true;

        /* log = "servos(t " +
                new Double((double)(long)(t_pi*100)/100).toString() + " b " +
                new Double((double)(long)(b_pi*100)/100).toString() + " e " +
                new Double((double)(long)(e_pi*100)/100) +")";
        log += " mm(x " +
                new Double((double)(long)(x*100)/100).toString() + " y " +
                new Double((double)(long)(y*100)/100).toString() + " z " +
                new Double((double)(long)(z*100)/100) +")"; */

    }


    public void solve_XYZ( double x, double y, double z ){

        x = Range.clip( x, xMin, xMax );
        y = Range.clip( y, xMin, yMax );
        z = Range.clip( z, zMin, zMax );

        r = Math.sqrt( x * x + y * y + z * z );
        if( r > rMax ) r = rMax;

        teta = Math.acos( z / r );
        phi = Math.atan( y / x );

        Triangle elbowTriangle = new Triangle();
        elbowTriangle.solve_SSS( lBase, lElbow, r );

        turretAngle.solve_anglePI( Math.PI - teta );
        baseAngle.solve_anglePI( Math.PI / 2 + phi );
        elbowAngle.solve_anglePI( elbowTriangle.a3 );

        isInitialized = true;
    }
}