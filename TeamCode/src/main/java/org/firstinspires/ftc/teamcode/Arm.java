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

    private double teta = 0;
    private double phi = 0;

    private double r = 0;

    public Angle  turretAngle          =  null;
    public Angle  baseAngle            =  null;
    public Angle  elbowAngle           =  null;

    public Angle  wristVerticalAngle    =  null;
    public Angle  wristHorizontalAngle  =  null;
    public Angle  clawOpeningAngle     =  null;
    public double clawOpeningMM        = 0;

    public Angle  turretAngleTest          =  null;
    public Angle  baseAngleTest            =  null;
    public Angle  elbowAngleTest           =  null;

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
    }

    public void Init(){

        turretAngle          =  new Angle();
        baseAngle            =  new Angle();
        elbowAngle           =  new Angle();

        wristVerticalAngle    =  new Angle();
        wristHorizontalAngle  =  new Angle();
        clawOpeningAngle     =  new Angle();

        turretAngleTest          =  new Angle();
        baseAngleTest            =  new Angle();
        elbowAngleTest           =  new Angle();

        turretAngle.Init( 0.0, 1.0, 0.05, 0.95 );
        baseAngle.Init( 0.0, 1.0, 0.05, 0.95 );
        elbowAngle.Init( 0.0, 1.0, 0.05, 0.95 );

        wristVerticalAngle.Init( 0.0, 1.0, 0.05, 0.95 );
        wristHorizontalAngle.Init( 0.0, 1.0, 0.05, 0.95 );
        clawOpeningAngle.Init( 0.0, 1.0, 0.05, 0.95 );

        clawOpeningMM = 0;

        turretAngleTest.Init( 0.0, 1.0, 0.05, 0.95 );
        baseAngleTest.Init( 0.0, 1.0, 0.05, 0.95 );
        elbowAngleTest.Init( 0.0, 1.0, 0.05, 0.95 );

        setClaw( lClawGap );
    }

    public double getX(){ return x; }
    public double getY(){ return y; }
    public double getZ(){ return z; }

    public double getTestTurretServo(){ return turretAngleTest.getServo(); }
    public double getTestBaseServo(){ return baseAngleTest.getServo(); }
    public double getTestElbowServo(){ return elbowAngleTest.getServo(); }

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

    public double getBaseServo(){ return baseAngle.getServo(); }
    public double getTurretServo(){ return turretAngle.getServo(); }
    public double getElbowServo(){ return elbowAngle.getServo(); }
    public double getWristHorizontalServo(){ return wristHorizontalAngle.getServo(); }
    public double getWristVerticalServo(){ return wristVerticalAngle.getServo(); }
    public double getClawServo(){ return clawOpeningAngle.getServo(); }

    public void copyFrom( Arm anotherArm ){
        setXYZ( anotherArm.x, anotherArm.y, anotherArm.z  );
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

        turretAngle.setPI( ts );
        baseAngle.setPI( bs );
        elbowAngle.setPI( es );

        double sign_x = 1.0;
        double sign_y = 1.0;
        double sign_z = 1.0;

        teta = turretAngle.getPI() - Math.PI/2;

        Triangle elbowTriangle = new Triangle();
        elbowTriangle.solve_SSA( lBase, lElbow, elbowAngle.getPI() );
        phi = Math.PI / 2 - ( baseAngle.getPI() - elbowTriangle.a2 );

        r = elbowTriangle.l3;

        double t = teta;
        double p = phi;

        z = r * Math.cos(phi); // works > pi/2 and < 0
        x = r * Math.sin(phi) * Math.cos(teta);
        y = r * Math.sin(phi) * Math.sin(teta);
    }

    public void setXYZ( double x, double y, double z ){

        x = Range.clip( x, xMin, xMax );
        y = Range.clip( y, xMin, yMax );
        z = Range.clip( z, zMin, zMax );

        r = Math.sqrt( x * x + y * y + z * z );

        double sign_x = x > 0 ? 1.0 : -1.0;
        double sign_y = y > 0 ? 1.0 : -1.0;
        double sign_z = z > 0 ? 1.0 : -1.0;

        phi = Math.acos( z * sign_z / r );
        teta = Math.atan( y * sign_y / x * sign_x );

        Triangle elbowTriangle = new Triangle();
        elbowTriangle.solve_SSS( lBase, lElbow, r );

        double t = Math.PI /2 + teta;
        if( x < 0 && teta < Math.PI / 2 ) t = Math.PI / 2 - teta;
        turretAngle.setPI( t );

        if( y < 0 )phi *= -1;
        if( z < 0 && phi < Math.PI / 2  )phi += Math.PI / 2;

        baseAngle.setPI( Math.PI / 2 + phi );
        elbowAngle.setPI( elbowTriangle.a3 );
    }

    public void testXYZ( double x, double y, double z ){

        double xTest = Range.clip( x, xMin, xMax );
        double yTest = Range.clip( y, xMin, yMax );
        double zTest = Range.clip( z, zMin, zMax );

        double rTest = Math.sqrt( xTest * xTest + yTest * yTest + zTest * zTest );

        double sign_x = xTest > 0 ? 1.0 : -1.0;
        double sign_y = yTest > 0 ? 1.0 : -1.0;
        double sign_z = zTest > 0 ? 1.0 : -1.0;

        double phiTest = Math.acos( zTest * sign_z / rTest );
        double tetaTest = Math.atan( yTest * sign_y / xTest * sign_x );

        Triangle elbowTriangle = new Triangle();
        elbowTriangle.solve_SSS( lBase, lElbow, rTest );

        double tTest = Math.PI /2 + tetaTest;
        if( xTest < 0 && tetaTest < Math.PI / 2 ) tTest = Math.PI / 2 - tetaTest;
        turretAngleTest.setPI( tTest );

        if( yTest < 0 )phiTest *= -1;
        if( zTest < 0 && phiTest < Math.PI / 2  )phiTest += Math.PI / 2;

        baseAngleTest.setPI( Math.PI / 2 + phiTest );
        elbowAngleTest.setPI( elbowTriangle.a3 );
    }
}