package org.firstinspires.ftc.teamcode;

import android.animation.ValueAnimator;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;

/**
 * Created by MuchHat on 9/25/2017.
 */

public class SafeServo {

    /* local OpMode members. */
    private ElapsedTime runtime  = new ElapsedTime();
    private Servo theServo = null;

    private double A0_255 = 0;
    private double A180_255 = 255;
    private double minLimit_255 = 0;
    private double maxLimit_255 = 255;
    private double homePos_255 = 128;
    private double direction = 1.0;
    private double position = 99;

    private double lastMovePos = 0;
    private double lastMoveTime = 0;

    /* Constructor */
    public void SafeServo(){
    }

    public void init( Servo aServo ){
        theServo = aServo;
        runtime.reset();
    }

    public void configLimits( double min_255, double max_255, double a0_255, double a180_255) {

        minLimit_255 = Range.clip( min_255, 0, 255 );
        maxLimit_255 =  Range.clip( max_255, 0, 255 );
        A0_255 =  Range.clip( a0_255, 0, 255 );
        A180_255 =  Range.clip( a180_255, 0, 255 );

        if( a0_255 > a180_255 ) {
            direction = -1.0;
        }
        if( a180_255 > a0_255 ) {
            direction = 1.0;
        }
    }

    public void configHome ( double home ) {
        homePos_255 = Range.clip( home, minLimit_255, maxLimit_255 );
        }

    public double getMinLimit() {
        return Range.clip( ( minLimit_255 - A0_255 ) / ( A180_255 - A0_255 ), 0.0, 1.0 );
    }

    public double getHomePosIncrement() {
        double crrPos = theServo.getPosition();
        double homePos = ( homePos_255 - A0_255 ) / ( A180_255 - A0_255 );
        return Range.clip( ( homePos - crrPos ), 0.0, 1.0 );
    }

    public double getMaxLimit() {
        return Range.clip( ( maxLimit_255 - A0_255 ) / ( A180_255 - A0_255 ), 0.0, 1.0 );
    }

    public void setPositionHome() {

        double pos_1 = ( homePos_255 - A0_255 ) /  ( A180_255 - A0_255 );
        moveServoPosition( pos_1 );
    }

    public void setPosition( double pos ) {

        position = pos;
        theServo.setPosition( getAdjustedPositionSafe( pos ) );
    }

    public double getPosition() {

        return theServo.getPosition();
    }

    public double getAngle() {

        return ( theServo.getPosition() * 180 - ( A0_255 /255 ) * 180 ) * Math.PI;
    }

    public double getPosition_255() {

        return theServo.getPosition() * ( A180_255 - A0_255 ) + A0_255;
    }

    public void moveServoPosition( double pos ) {

        double pos_1 = getAdjustedPositionSafe( pos );

        theServo.setPosition( Range.clip( pos_1, 0.0, 1.0 ) );
    }

    public void moveServoIncremental( double posSpeed ) { // e.g. 1

        // 1 on the keypad means 1 increment every 33 millis

        double unitSize = 33;
        double increment = 0.05;

        // it needs to move at least posSpeed in an unit

        if( position > 1.0 || position < 0.0 ) {
            // first run
            position = theServo.getPosition();
            lastMovePos = position;
            runtime.reset();
            lastMoveTime = runtime.milliseconds() - unitSize;
        }

        double distanceTraveled = Math.abs( position - lastMovePos ); // e.g. 0
        double unitsPassed = ( runtime.milliseconds() - lastMoveTime ) / unitSize; // e.g 3
        double behind = unitsPassed * posSpeed - distanceTraveled; // e.g. 3

        if( Math.abs( behind ) > 0 ) {
            double moveNeeded = behind * 1.1; // 0.15

            if( moveNeeded > 11 || moveNeeded < -11 ){
                moveNeeded = 1;
                position = theServo.getPosition() + moveNeeded * increment;
                runtime.reset();
                lastMoveTime = runtime.milliseconds();
                lastMovePos = position;
            }
            else if( moveNeeded < 12 && moveNeeded > -12 ) {
                position = theServo.getPosition() + moveNeeded * increment;
                runtime.reset();
                lastMoveTime = runtime.milliseconds();
                lastMovePos = position;
            }
            double pos_1 = getAdjustedPositionSafe( position );
            theServo.setPosition(Range.clip(pos_1, 0.00, 1.00));
        }
    }

    double getAngleToHorizontal() {
        return 0;
    }

    double getAdjustedPositionSafe ( double pos ) {

        pos = Range.clip( pos, 0.0, 1.0 );

        double pos_255 = A0_255 + (A180_255 - A0_255) * pos;

        pos_255 = Range.clip( pos_255, minLimit_255, maxLimit_255 );

        double pos_1 = pos_255 / ( 255 );

        return Range.clip( pos_1, 0.0, 1.0 );
    }
}