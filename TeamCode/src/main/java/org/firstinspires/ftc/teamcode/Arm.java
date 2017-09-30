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
    private double a2 = 0;

    public Angle  turretAngle          =  null;
    public Angle  baseAngle            =  null;
    public Angle  elbowAngle           =  null;

    public Angle  wristVerticalAngle    =  null;
    public Angle  wristHorizontalAngle  =  null;
    public Angle  clawOpeningAngle      =  null;
    public double clawOpeningMM         = 0;

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

    double lBase     = 188; // mm
    double lElbow    = 233; // mm
    double lClawArm =  150; // mm
    double lClawGap =  37;  // mm

    double xMax = 450; // mm
    double xMin = -450; // mm
    double yMax = 450; // mm
    double yMin = 0; // mm
    double zMax = 450; // mm
    double zMin = -150; // mm
    double rMax = 450; // mm

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
        clawOpeningAngle      =  new Angle();

        turretAngle.Init( 0.0, 0.8, 0.05, 0.95 );
        baseAngle.Init( 0.0, 0.8, 0.33, 0.66 );
        elbowAngle.Init( 0.0, 0.8, 0.33, 0.66 );

        wristVerticalAngle.Init( 0.0, 1.0, 0.05, 0.95 );
        wristHorizontalAngle.Init( 0.0, 1.0, 0.05, 0.95 );
        clawOpeningAngle.Init( 0.0, 1.0, 0.05, 0.95 );

        clawOpeningMM = 0;

        setClaw( lClawGap );
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
    public double getA2(){ return a2; }

    public double getBaseServo(){ return baseAngle.getServo(); }
    public double getTurretServo(){ return turretAngle.getServo(); }
    public double getElbowServo(){ return elbowAngle.getServo(); }
    public double getWristHorizontalServo(){ return wristHorizontalAngle.getServo(); }
    public double getWristVerticalServo(){ return wristVerticalAngle.getServo(); }
    public double getClawServo(){ return clawOpeningAngle.getServo(); }

    public void copyFrom( Arm anotherArm ){
        setServos( anotherArm.getTurretServo(), anotherArm.getBaseServo(), anotherArm.getElbowServo()  );
        setClaw( anotherArm.clawOpeningMM );
    }

    public void setClaw( double mm ){

        Triangle clawTriangle = new Triangle();

        mm = Math.abs( mm );

        clawTriangle.solve_SSS( ( mm - lClawGap ) / 2, lClawArm,
                Math.sqrt( ( ( mm - lClawGap ) / 2 ) * ( ( mm - lClawGap ) / 2 ) +
                        lClawArm * lClawArm ) );
        clawOpeningAngle.setPI( clawTriangle.a1 );

        clawOpeningMM = mm;
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

        Triangle elbowTriangle = new Triangle();
        elbowTriangle.solve_SSA( lBase, lElbow, elbowAngle.getPI() );

        a2 = elbowTriangle.a2;

        phi = baseAngle.getPI() - Math.PI / 2 - elbowTriangle.a2  ;

        r = elbowTriangle.l3;

        z = r * Math.cos(phi); // works > pi/2 and < 0
        x = r * Math.sin(phi) * Math.cos(teta);
        y = r * Math.sin(phi) * Math.sin(teta);

        //Log.d( "MuchHat", String.format( "Arm_setServos xyz: %.3f, %.3f, %3.f ", x, y, z ) );
    }

    public void setXYZ( double ax, double ay, double az ){

        x = Range.clip( ax, xMin, xMax );
        y = Range.clip( ay, yMin, yMax );
        z = Range.clip( az, zMin, zMax );

        Log.d( "MuchHat", String.format( "setXYZ x: %.2f", x ) );
        Log.d( "MuchHat", String.format( "setXYZ y: %.2f", y ) );
        Log.d( "MuchHat", String.format( "setXYZ z: %.2f", z ) );

        r = Math.sqrt( x * x + y * y + z * z );
        if( r == 0 ) r = 0.001;

        Log.d( "MuchHat", String.format( "setXYZ r: %.2f", r ) );

        phi = Math.acos( z / r );

        Log.d( "MuchHat", String.format( "setXYZ phi: %.2f", phi ) );

        if( Math.abs( x ) <= 0.001 ){
            teta = Math.PI;
        }
        else if(Math.abs( x ) > 0.001 ){
            teta = Math.atan( Math.abs( y / x ) );
            if( x < 0 )teta += Math.PI / 2;
        }

        Log.d( "MuchHat", String.format( "setXYZ teta: %.2f", teta ) );

        Triangle elbowTriangle = new Triangle();
        elbowTriangle.solve_SSS( lBase, lElbow, r );

        a2 = elbowTriangle.a2;

        turretAngle.setPI( Math.PI - teta );
        baseAngle.setPI( Math.PI - phi + elbowTriangle.a2 );
        elbowAngle.setPI( elbowTriangle.a3 );
    }

}