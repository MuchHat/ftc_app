package org.firstinspires.ftc.teamcode;

/**
 * Created by gigela on 9/27/2017.
 */

public class ArmController {

    Arm origin = null;
    Arm current = null;
    Arm next = null;
    Arm destination = null;

    Claw clawOrigin = null;
    Claw clawDestination = null;

    double currentSpeed_mms = 0;
    double maxSpeed_mms = 0;
    double rampAccel_mmsmm = 0;

    double platformHeight = 155;
    double baseToEdgeX = 166;
    double baseToEdgeY = 122;
    double atDestinationTolerance = 3;

    boolean isInitialized = false;

    public void startLoop( double servoT, double servoB, double servoE, double servoW, double servoC ){
        // determines the current position

        if( !isInitialized ){
            destination.solve_XYZ( destination.xZero, destination.yZero, destination.zZero );
            origin.copyFrom( destination );
            current.copyFrom( destination );
            next.copyFrom( destination );

            isInitialized = true;
        }
        else if( isInitialized ){
            current.solve_Servos( servoT, servoB, servoE );
        }
    }

    public void endLoop( double stepMillis ) {
        // determined the next point based on the millis : considered end of step

        // compute the distance to the next point
        double distanceToDestination = current.wristPoint.distsanceTo( destination.wristPoint );

        // determine the max possible speed
        double curentAllowedMaxSpeed_mms = 11;
        double currentAllowedMaxDistance = curentAllowedMaxSpeed_mms * stepMillis;

        // see if the destination can be achieved with this speed if not adjust next
        if( distanceToDestination < atDestinationTolerance ) {
            next.copyFrom( current );
        }
        else if( distanceToDestination < currentAllowedMaxDistance ){
            next.copyFrom( destination );
        }
        else if( distanceToDestination > currentAllowedMaxDistance ) {
            double ratio = currentAllowedMaxDistance / distanceToDestination;
            next.solve_XYZ( destination.wristPoint.x * ratio,
                    destination.wristPoint.y * ratio,
                    destination.wristPoint.z * ratio );
        }

        // see if would hit the body of the robot and adjust
        if( next.wristPoint.x < baseToEdgeX &&
                next.wristPoint.y < baseToEdgeY &&
                next.wristPoint.z < platformHeight && ){
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


    }

    public void moveIncremental( double ix, double iy, double iz ){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop
    }

    public void moveToPosition( double ax, double ay, double az ){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop

        isInitialized = true;
    }

    public void moveToPositionZero(){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop

        isInitialized = true;
    }

    public void moveToPositionHome(){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop

        isInitialized = true;
    }

    public void moveToPositionFront(){

        // use this to set the new elbow, wrist, claw to the pos that will be read to set the servos in a loop

        isInitialized = true;
    }
    public void check(){

        // adjust for not bumping in the chasis
    }
}
