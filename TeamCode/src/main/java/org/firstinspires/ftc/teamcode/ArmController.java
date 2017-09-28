package org.firstinspires.ftc.teamcode;

/**
 * Created by gigela on 9/27/2017.
 */

public class ArmController {

    Arm origin = null;
    Arm current = null;
    Arm next = null;
    Arm destination = null;

    double maxSpeed_mms = 33;
    double maxAccel_mmss = 66;
    double prevSpeed_mms = 0;

    double platformHeight = 155;
    double baseToEdgeX = 166;
    double baseToEdgeY = 122;
    double atDestinationTolerance = 3;

    boolean isInitialized = false;

    public void startLoop( double servoTurret, double servoBase, double servoElbow ){
        // determines the current position

        if( !isInitialized ){
            destination.solve_XYZ( destination.xZero, destination.yZero, destination.zZero );
            origin.copyFrom( destination );
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
        double distanceToDestination = current.wristPoint.distsanceTo( destination.wristPoint );

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
                    current.wristPoint.x + ( destination.wristPoint.x - current.wristPoint.x ) * ratio,
                    current.wristPoint.y + ( destination.wristPoint.y - current.wristPoint.y ) * ratio,
                    current.wristPoint.z + ( destination.wristPoint.z - current.wristPoint.z ) * ratio );
        }

        // check the speed of the claw too
        double clawDistance = destination.clawOpeningMM - current.clawOpeningMM;
        double currentAllowedDistance =  newSpeed_mms * stepMillis;
        if( Math.abs( clawDistance ) > maxSpeed_mms ){
            double ratio = Math.abs( currentAllowedMaxDistance / clawDistance );
            next.clawOpeningMM = current.clawOpeningMM + ( destination.clawOpeningMM - current.clawOpeningMM ) * ratio;
        }

        // see if would hit the body of the robot and adjust
        if( next.wristPoint.x < baseToEdgeX &&
                next.wristPoint.y < baseToEdgeY &&
                next.wristPoint.z < platformHeight ) {
            // means it would hit the robot

            // keep constant the one closer to the arm
            double dx = baseToEdgeX - next.wristPoint.x;
            double dy = baseToEdgeY - next.wristPoint.y;
            double dz = platformHeight - next.wristPoint.z;

            if( dx < dy && dx < dz ){
                next.solve_XYZ( baseToEdgeX, next.wristPoint.y, next.wristPoint.z );
            }
            if( dy < dx && dy < dz ){
                next.solve_XYZ( next.wristPoint.x, baseToEdgeY, next.wristPoint.z );
            }
            if( dz < dx && dz < dy ){
                next.solve_XYZ( next.wristPoint.x, next.wristPoint.y, platformHeight );
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
