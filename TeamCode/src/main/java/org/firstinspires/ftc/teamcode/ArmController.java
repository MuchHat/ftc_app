package org.firstinspires.ftc.teamcode;

/**
 * Created by gigela on 9/27/2017.
 */

public class ArmController {

    Arm current     = new Arm();
    Arm next        = new Arm();
    Arm destination = new Arm();

    // TODO fix constants
    double maxSpeed_mms  = 33; // mm
    double maxAccel_mmss = 66; // mm
    double prevSpeed_mms = 0;  // mm

    double platformHeight = 155; // mm
    double baseToEdgeX    = 166; // mm
    double baseToEdgeY    = 122; // mm
    double closestX       = 66; // mm
    double closestY       = 22; // mm

    double atDestinationTolerance = 3; // mm
    // TODO end

    boolean isInitialized = false;

    public void startLoop( double servoTurret, double servoBase, double servoElbow ){
        // determines the current position

        if( !isInitialized ){
            //destination.solve_XYZ( destination.xZero, destination.yZero, destination.zZero );
            current.copyFrom( destination );
            next.copyFrom( destination );

            isInitialized = true;
        }
        else if( isInitialized ){
            current.solve_Servos( servoTurret, servoBase, servoElbow );
        }
    }

    public void endLoop( double stepMillis ) {
        // determined the next point based on the millis : considered end of step

        // compute the distance to the next point
        double distanceToDestination = current.endPoint.distanceTo( destination.endPoint );

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
        double currentAllowedMaxDistance = newSpeed_mms * stepMillis;

        // see if the destination can be achieved with this speed if not adjust next
        if( distanceToDestination < atDestinationTolerance ) {
            next.copyFrom( current );
            prevSpeed_mms = 0;
        }
        else if( Math.abs( distanceToDestination ) < currentAllowedMaxDistance ){
            next.copyFrom( destination );
        }
        else if( distanceToDestination > currentAllowedMaxDistance ) {
            double ratio = Math.abs( currentAllowedMaxDistance / distanceToDestination );
            next.solve_XYZ(
                    current.endPoint.x + ( destination.endPoint.x - current.endPoint.x ) * ratio,
                    current.endPoint.y + ( destination.endPoint.y - current.endPoint.y ) * ratio,
                    current.endPoint.z + ( destination.endPoint.z - current.endPoint.z ) * ratio );
        }

        // check the speed of the claw too
        double clawDistance = destination.clawOpeningMM - current.clawOpeningMM;
        double currentAllowedDistance =  newSpeed_mms * stepMillis;
        if( Math.abs( clawDistance ) > maxSpeed_mms ){
            double ratio = Math.abs( currentAllowedMaxDistance / clawDistance );
            next.clawOpeningMM = current.clawOpeningMM + ( destination.clawOpeningMM - current.clawOpeningMM ) * ratio;
        }

        // check if too close to base
        // see if would hit the body of the robot and adjust
        if( next.endPoint.x < closestX ) {
            next.solve_XYZ(closestX, next.endPoint.y, next.endPoint.z);
        }
        if( next.endPoint.y < closestY ) {
            next.solve_XYZ(next.endPoint.x, closestY, next.endPoint.z);
        }
        // see if would hit the body of the robot and adjust
        if( next.endPoint.x < baseToEdgeX &&
                next.endPoint.y < baseToEdgeY &&
                next.endPoint.z < platformHeight ) {
            // means it would hit the robot

            // keep constant the one closer to the arm
            double dx = baseToEdgeX - next.endPoint.x;
            double dy = baseToEdgeY - next.endPoint.y;
            double dz = platformHeight - next.endPoint.z;

            if( dx < dy && dx < dz ){
                next.solve_XYZ( baseToEdgeX, next.endPoint.y, next.endPoint.z );
            }
            if( dy < dx && dy < dz ){
                next.solve_XYZ( next.endPoint.x, baseToEdgeY, next.endPoint.z );
            }
            if( dz < dx && dz < dy ){
                next.solve_XYZ( next.endPoint.x, next.endPoint.y, platformHeight );
            }
        }
        prevSpeed_mms = newSpeed_mms;
    }

    public void moveIncremental( double ix, double iy, double iz ){

        destination.solve_XYZ( current.x + ix, current.y + iy, current.z + iz );
    }

    public void moveToPosition( double ax, double ay, double az ){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop
        destination.solve_XYZ( ax, ay, az );

        isInitialized = true;
    }

    public void clawIncremental( double ix ){
        destination.solve_Claw( current.clawOpeningMM + ix );
    }

    public void clawToPosition( double x ){
        destination.solve_Claw( x );
    }

    public void moveToPositionZero(){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop
        destination.solve_XYZ( destination.xZero, destination.yZero, destination.zZero );

        isInitialized = true;
    }

    public void moveToPositionHome(){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop
        destination.solve_XYZ( destination.xHome, destination.yHome, destination.zHome );

        isInitialized = true;
    }

    public void moveToPositionFront(){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop
        destination.solve_XYZ( destination.xFront, destination.yFront, destination.zFront );

        isInitialized = true;
    }
}
