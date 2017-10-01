package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by gigela on 9/27/2017.
 */

public class ArmController {

    public Arm current     = new Arm();
    public Arm next        = new Arm();
    public Arm destination = new Arm();
    public Arm test        = new Arm();

    // TODO fix constants
    double maxSpeed_mms  = 1111; // mm
    double minSpeed_mms  = 444; // mm
    double maxAccel_mmss = 1111; // mm
    double prevSpeed_mms = 0;  // mm

    double platformHeight = 155; // mm
    double baseToEdgeX    = 166; // mm
    double baseToEdgeY    = 122; // mm
    double closestX       = 66; // mm
    double closestY       = 22; // mm

    boolean atDestination = true;
    boolean isInitialized = false;

    public double distanceToDestination = 0;
    public double distanceToNext = 0;

    double atDestinationTolerance = 10; // mm
    // TODO end

    public void ArmController(){
    }

    public void init(){
        current.init();
        next.init();
        destination.init();
        test.init();
    }

    public void setDestinationToCurrent( double aServoTurret, double aServoBase, double aServoElbow ){
        current.setServos( aServoTurret, aServoBase, aServoElbow );
        destination.copyFrom( current );
        next.copyFrom( current );
    }

    public void startLoop( double aServoTurret, double aServoBase, double aServoElbow ){
        // determines the current position

        current.setServos( aServoTurret, aServoBase, aServoElbow );
        test.setXYZ( current.getX(), current.getY(), current.getZ() );

        //Log.d( "MuchHat", String.format( "startLoop_setServos: %.3f, %.3f, %3.f ", aServoTurret, aServoBase, aServoElbow ) );
        //Log.d( "MuchHat", String.format( "startLoop_currentServos: %.3f, %.3f, %3.f ", current.getTurretServo(), current.getBaseServo(), current.getElbowServo() ) );


        if( !isInitialized ){
            destination.copyFrom( current );
            next.copyFrom( current );
            isInitialized = true;
        }
    }

    public void endLoop( double stepMillis ) {
        // determined the next point based on the millis : considered end of step

        // compute the distance to the next point
        distanceToDestination = current.distanceTo( destination );

        //Log.d( "MuchHat", String.format( "endLoop_distanceToDestination: %.3f ", distanceToDestination ) );
        //Log.d( "MuchHat", String.format( "endLoop_current: %.3f, %.3f, %3.f ", current.getX(), current.getY(), current.getZ() ) );
        //Log.d( "MuchHat", String.format( "endLoop_destination: %.3f, %.3f, %3.f ", destination.getX(), destination.getY(), destination.getZ() ) );

        // determine the max possible speed
        double newSpeed_mms = maxSpeed_mms;

        // ensure does not exceed acceleration
        if( newSpeed_mms > prevSpeed_mms + maxAccel_mmss * stepMillis ){
            newSpeed_mms = prevSpeed_mms + maxAccel_mmss * stepMillis;
        }
        // decelerate at the end
        double maxCurrentSpeedToDecelerateToDestination = Math.sqrt( 2 * maxAccel_mmss * distanceToDestination );
        if( newSpeed_mms > maxCurrentSpeedToDecelerateToDestination ){
            newSpeed_mms = maxCurrentSpeedToDecelerateToDestination;
        }
        newSpeed_mms = Range.clip( newSpeed_mms, minSpeed_mms, maxSpeed_mms );
        double currentAllowedMaxDistance = newSpeed_mms * stepMillis;
        if( currentAllowedMaxDistance <  atDestinationTolerance ) currentAllowedMaxDistance = atDestinationTolerance * 2;

        // by default stay in place
        atDestination = true;
        next.copyFrom( current );

        // see if the destination can be achieved with this speed if not adjust next
        if( ( Math.abs( distanceToDestination ) <= atDestinationTolerance ) ||
                ( ( Math.abs( current.getTurretServo() - destination.getTurretServo() ) <= 0.05 ) &&
                        ( ( Math.abs( current.getBaseServo() - destination.getBaseServo() ) ) <= 0.05 ) &&
                                ( ( Math.abs( current.getElbowServo() - destination.getElbowServo() ) ) <= 0.05 ) ) ){
            next.copyFrom( current );
            destination.copyFrom( current );
            prevSpeed_mms = 0;
            atDestination = true;
        }
        else if( Math.abs( distanceToDestination ) <= currentAllowedMaxDistance ){
            next.copyFrom( destination );
            prevSpeed_mms = newSpeed_mms;
            atDestination = false;
        }
        else if( distanceToDestination > currentAllowedMaxDistance ) {
            double ratio = Math.abs( currentAllowedMaxDistance / distanceToDestination );
            //Log.d( "MuchHat", String.format( "ratio: %.3f ", ratio ) );
            next.setXYZ(
                    current.getX() + ( destination.getX() - current.getX() ) * ratio,
                    current.getY() + ( destination.getY() - current.getY() ) * ratio,
                    current.getZ() + ( destination.getZ() - current.getZ() ) * ratio );
            prevSpeed_mms = newSpeed_mms;
            atDestination = false;
        }
        distanceToNext = current.distanceTo( next );

        // check the speed of the claw too
        double clawDistance = Math.abs( destination.clawOpeningMM - current.clawOpeningMM );
        double currentAllowedDistance =  newSpeed_mms * stepMillis;
        if( Math.abs( clawDistance ) > maxSpeed_mms ){
            double ratio = Math.abs( currentAllowedMaxDistance / clawDistance );
            next.clawOpeningMM = current.clawOpeningMM + ( destination.clawOpeningMM - current.clawOpeningMM ) * ratio;
        }

        //Log.d( "MuchHat", String.format( "endLoop_destination: %.3f, %.3f, %3.f ", next.getX(), next.getY(), next.getZ() ) );

        // check if too close to base
        // see if would hit the body of the robot and adjust
        if( next.getX() < closestX ) {
            next.setXYZ(closestX, next.getY(), next.getZ());
        }
        if( next.getY() < closestY ) {
            next.setXYZ(next.getX(), closestY, next.getZ());
        }
        // see if would hit the body of the robot and adjust
        if( next.getX() < baseToEdgeX &&
                Math.abs( next.getY() ) < baseToEdgeY &&
                next.getZ() < platformHeight ) {
            // means it would hit the robot

            double x_sign = next.getX() > 0 ? 1.0 : -1.0;

            // keep constant the one closer to the destination
            next.setXYZ( baseToEdgeX * x_sign, next.getY(), next.getZ() );
            double dx = next.distanceTo( destination );

            next.setXYZ( next.getX(), baseToEdgeY, next.getZ() );
            double dy = next.distanceTo( destination );

            next.setXYZ( next.getX(), next.getY(), platformHeight );
            double dz = next.distanceTo( destination );

            if( dx < dy && dx < dz ){
                next.setXYZ( baseToEdgeX * x_sign, next.getY(), next.getZ() );
            }
            if( dy < dx && dy < dz ){
                next.setXYZ( next.getX(), baseToEdgeY, next.getZ() );
            }
            if( dz < dx && dz < dy ){
                next.setXYZ( next.getX(), next.getY(), platformHeight );
            }
        }
        prevSpeed_mms = newSpeed_mms;
        //Log.d( "MuchHat", String.format( "endLoop_newSpeed_mms: %.3f ", newSpeed_mms ) );
    }

    public double getNextBaseServo(){ return next.getBaseServo(); }
    public double getNextElbowServo(){ return next.getElbowServo(); }
    public double getNextWristHorizontalServo(){ return next.getWristHorizontalServo(); }
    public double getNextTurretServo(){ return next.getTurretServo(); }
    public double getNextWristVerticalServo(){ return next.getWristVerticalServo(); }
    public double getNextClawServo(){ return next.getClawServo(); }

    public double getCurrentTurretServo(){ return current.getTurretServo(); }
    public double getCurrentBaseServo(){ return current.getBaseServo();}
    public double getCurrentElbowServo(){ return current.getElbowServo();}

    public double getCurrentR(){ return current.getR(); }
    public double getCurrentTeta(){ return current.getTeta(); }
    public double getCurrentPhi(){ return current.getPhi(); }
    public double getCurrentA2(){ return current.getA2(); }

    public double getCurrentX(){ return current.getX(); }
    public double getCurrentY(){ return current.getY(); }
    public double getCurrentZ(){ return current.getZ(); }

    public double getCurrentTestX(){ return test.getX(); }
    public double getCurrentTestY(){ return test.getY() ; }
    public double getCurrentTestZ(){ return test.getZ(); }

    public double getCurrentTestR(){ return test.getR(); }
    public double getCurrentTestPhi(){ return test.getPhi(); }
    public double getCurrentTestTeta(){ return test.getTeta(); }
    public double getCurrentTestA2(){ return test.getA2(); }

    public double getCurrentTestTurretServo(){ return test.getTurretServo(); }
    public double getCurrentTestBaseServo(){ return test.getBaseServo(); }
    public double getCurrentTestELbowServo(){ return test.getElbowServo(); }

    public void moveIncremental( double ix, double iy, double iz ){

        destination.setXYZ( current.getX() + ix, current.getY() + iy, current.getZ() + iz );
    }

    public void moveToPosition( double ax, double ay, double az ){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop
        destination.setXYZ( ax, ay, az );

        isInitialized = true;
    }

    public void clawIncremental( double ix ){
        destination.setClaw( current.clawOpeningMM + ix );
    }

    public void clawToPosition( double x ){
        destination.setClaw( x );
    }

    public void moveToPositionZero(){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop
        destination.setXYZ( destination.getZeroX(), destination.getZeroY(), destination.getZeroZ() );

        isInitialized = true;
    }

    public void moveToPositionHome(){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop
        destination.setXYZ( destination.getHomeX(), destination.getHomeY(), destination.getHomeZ() );

        isInitialized = true;
    }

    public void moveToPositionFront(){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop
        destination.setXYZ( destination.getFrontX(), destination.getFrontY(), destination.getFrontZ() );

        isInitialized = true;
    }
}
