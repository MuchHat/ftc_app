package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Created by gigela on 9/27/2017.
 */

public class Arm {

    private double x = 0;
    private double y = 0;
    private double z = 0;

    private double teta = 0;
    private double phi = 0;
    private double r = 0;
    private double e_a2 = 0;

    private double clawMM = 0;

    public Angle  turretAngle          =  null;
    public Angle  baseAngle            =  null;
    public Angle  elbowAngle           =  null;

    public Angle  wristVerticalAngle    =  null;
    public Angle  wristHorizontalAngle  =  null;
    public Angle  clawRightAngle        =  null;
    public Angle  clawLeftAngle        =  null;

    // TODO correct the home positons
    double xZero = 0; // mm
    double yZero = 22; // mm
    double zZero = 222; // mm

    double xHome = 0; // mm
    double yHome = 22; // mm
    double zHome = 222; // mm

    double xFront = 0; // mm
    double yFront = 111; // mm
    double zFront = 222; // mm

    double mmClawOpen  = 222;  // mm
    double mmClawClose = 188; // mm

    double lBase     = 188; // mm
    double lElbow    = 233; // mm
    double lClawArm =  150; // mm
    double lClawGap =  37;  // mm

    double xMax = 333; // mm
    double xMin = -333; // mm
    double yMax = 333; // mm
    double yMin = 0; // mm
    double zMax = 333; // mm
    double zMin = -111; // mm
    double rMax = ( lBase + lElbow ) * 0.95; // mm
    double rMin = 11;

    boolean initTable = false;

    // TODO END

    boolean isInitialized = false;
    String log = new String();

    /* Constructor */
    public void Arm(){
    }

    public void init(){

        turretAngle          =  new Angle();
        baseAngle            =  new Angle();
        elbowAngle           =  new Angle();

        wristVerticalAngle    =  new Angle();
        wristHorizontalAngle  =  new Angle();
        clawRightAngle        =  new Angle();
        clawLeftAngle         =  new Angle();

        turretAngle.Init_45_135( 0.140, 0.644, 0.05, 0.897 ); // turret setup

        baseAngle.Init_45_135( 0.25, 0.75, 0.05, 0.95 ); // TODO
        elbowAngle.Init_45_135( 0.404, 0.950, 0.20, 0.95 ); //elbow setup

        wristVerticalAngle.Init_45_135( 1.1775, 0.6105, 0.32, 0.89 );
        clawRightAngle.Init_45_135( 0.221, -0.437, 0.221, 0.55 ); // right claw setup
        clawLeftAngle.Init_45_135( 0.818, 1.582, 0.436, 0.818 ); // left claw setup

        wristVerticalAngle.Init_45_135( 0.25, 0.75, 0, 1 ); // not a real servo, a virtual servo

        setClawMM( lClawGap );
    }

    public double getX(){ return x; }
    public double getY(){ return y; }
    public double getZ(){ return z; }

    public double getZeroX(){ return xZero; }
    public double getZeroY(){ return yZero; }
    public double getZeroZ(){ return zZero; }

    public double getHomeX(){ return xHome; }
    public double getHomeY(){ return yHome; }
    public double getHomeZ(){ return zHome; }

    public double getFrontX(){ return xFront; }
    public double getFrontY(){ return yFront; }
    public double getFrontZ(){ return zFront; }

    public double getR(){ return r; }
    public double getTeta(){ return teta; }
    public double getPhi(){ return phi; }
    public double getA2(){ return e_a2; }

    public double getBaseServo(){ return baseAngle.getServo(); }
    public double getTurretServo(){ return turretAngle.getServo(); }
    public double getElbowServo(){ return elbowAngle.getServo(); }
    public double getWristHorizontalServo(){ return wristHorizontalAngle.getServo(); }
    public double getWristVerticalServo(){ return wristVerticalAngle.getServo(); }
    public double getClawRightServo(){ return clawRightAngle.getServo(); }
    public double getClawLeftServo(){ return clawLeftAngle.getServo(); }

    public void copyFrom( Arm anotherArm ){
        setXYZ( anotherArm.getX(), anotherArm.getY(), anotherArm.getZ()  );
        setClawMM( anotherArm.getClawMM() );
    }

    public double getClawMM(){
        return clawMM;
    }

    public void setClawMM( double amm ){

        TriangleCalculator clawTriangle = new TriangleCalculator();

        amm = Math.abs( amm );

        double l1 = ( amm - lClawGap ) / 2;
        double l3 = lClawArm;

        if( l3 < l1 * 0.85 ) l3 = l1 * 0.88; // such it does not open too much

        double l2 = Math.sqrt( l3 * l3 - l1 * l1  );

        clawTriangle.setSSS( l1, l2, l3 );
        clawRightAngle.setPI( clawTriangle.a1 );
        clawLeftAngle.setPI( clawTriangle.a1 );

        clawMM = amm;
    }

    public double distanceTo( Arm anotherArm ){

        return Math.sqrt( ( x - anotherArm.getX() ) * ( x - anotherArm.getX() ) +
                ( y - anotherArm.getY() ) * ( y - anotherArm.getY() ) +
                ( z - anotherArm.getZ() ) * ( z - anotherArm.getZ() ) );

    }

    public void setServos( double ts, double bs, double es ) {

        turretAngle.setServo( ts );
        baseAngle.setServo( bs );
        elbowAngle.setServo( es );

        //Log.d( "MuchHat", String.format( "Arm_setServos servos: %.3f, %.3f, %3.f ", ts, bs, es ) );

        teta = Math.PI - turretAngle.getPI();

        TriangleCalculator elbowTriangle = new TriangleCalculator();
        elbowTriangle.setSSA( lBase, lElbow, elbowAngle.getPI() );

        e_a2 = elbowTriangle.a2;

        phi = baseAngle.getPI() - Math.PI / 2 + elbowTriangle.a2  ;

        r = elbowTriangle.l3;

        z = Range.clip( r * Math.cos(phi), zMin, zMax ) ; // works > pi/2 and < 0
        x = Range.clip( r * Math.sin(phi) * Math.cos(teta), xMin, xMax );
        y = Range.clip( r * Math.sin(phi) * Math.sin(teta), yMin, yMax );

        //Log.d( "MuchHat", String.format( "Arm_setServos xyz: %.3f, %.3f, %3.f ", x, y, z ) );
    }

    public void setXYZ( double ax, double ay, double az ){

        x = Range.clip( ax, xMin, xMax );
        y = Range.clip( ay, yMin, yMax );
        z = Range.clip( az, zMin, zMax );

        //Log.d( "MuchHat", String.format( "setXYZ x: %.2f", x ) );
        //Log.d( "MuchHat", String.format( "setXYZ y: %.2f", y ) );
        //Log.d( "MuchHat", String.format( "setXYZ z: %.2f", z ) );

        r = Range.clip( Math.sqrt( x * x + y * y + z * z ), rMin, rMax );

        //Log.d( "MuchHat", String.format( "setXYZ r: %.2f", r ) );

        phi = Math.acos( z / r );

        //Log.d( "MuchHat", String.format( "setXYZ phi: %.2f", phi ) );

        if( Math.abs( x ) <= 0.001 ){
            teta = Math.PI/2;
        }
        else if(Math.abs( x ) > 0.001 ){
            teta = Math.atan( Math.abs( y / x ) );
            if( x < 0 )teta += Math.PI / 2;
        }

        //Log.d( "MuchHat", String.format( "setXYZ teta: %.2f", teta ) );

        TriangleCalculator elbowTriangle = new TriangleCalculator();
        elbowTriangle.setSSS( lBase, lElbow, r );

        e_a2 = elbowTriangle.a2;

        turretAngle.setPI( Math.PI - teta );
        baseAngle.setPI( Math.PI - phi + elbowTriangle.a2 );
        elbowAngle.setPI( elbowTriangle.a3 );
    }

}