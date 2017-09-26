package org.firstinspires.ftc.teamcode;

import android.animation.ValueAnimator;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

    /* Constructor */
    public void SafeServo(){
    }

    public void init( Servo aServo ){
        theServo = aServo;
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

    public double getMaxLimit() {
        return Range.clip( ( maxLimit_255 - A0_255 ) / ( A180_255 - A0_255 ), 0.0, 1.0 );
    }

    public void setPositionHome() {

        double pos_1 = ( homePos_255 - A0_255 ) /  ( A180_255 - A0_255 );
        moveServo( pos_1 );
    }

    public void setPosition( double pos ) {

        theServo.setPosition( getAdjustedPositionSafe( pos ) );
    }

    public double getPosition() {

        return theServo.getPosition();
    }
    public double getPosition_255() {

        return theServo.getPosition() * ( A180_255 - A0_255 ) + A0_255;
    }

    public void moveServo( double pos ) {

        double pos_1 = getAdjustedPositionSafe( pos );
        double pos_crr = theServo.getPosition();

        double stepSize  = 0.005;
        double stepWait  = 0.2; // secs in a step
        double rampRatio = 0.3; // increase/decrease time per step during ramp

        double stepCount = Math.abs( pos_1 - pos_crr ) / stepSize;
        if( stepCount < 3 )stepCount = 3;
        double rampSteps = stepCount * 0.3; // 33% ramp up/down
        if( rampSteps < 1 )rampSteps = 1;
        if( rampSteps > stepCount / 2 ) rampSteps = stepCount / 2;

        runtime.reset();
        double waitTarget = 0;
        for( int i = 0; i < stepCount; i++ ){

            waitTarget += stepWait;

            double rampPosition = Math.max( rampSteps - i, i - ( stepCount - rampSteps )  );
            rampPosition = Math.max( rampPosition, 0 );
            rampPosition = Math.min( rampPosition, rampSteps );

            waitTarget += stepWait * rampRatio * rampPosition;

            theServo.setPosition( pos_crr + stepSize * ( pos_1 - pos_crr ) );

           while( runtime.seconds() < waitTarget ){

           }
        }
        theServo.setPosition( Range.clip( pos_1, 0.0, 1.0 ) );
    }
/*
        double animationTime = (long)( Math.abs( pos - theServo.getPosition() ) * 1666 );

        ValueAnimator valueAnimator = ValueAnimator.ofInt((int)( theServo.getPosition() * 1000 ), (int)( getAdjustedPositionSafe( pos ) * 1000 ) );
        valueAnimator.setDuration( 2222 );
        valueAnimator.addUpdateListener(new ValueAnimator.AnimatorUpdateListener() {
            @Override
            public void onAnimationUpdate(ValueAnimator valueAnimator) {
                double crrPos = Double.valueOf(valueAnimator.getAnimatedValue().toString()) / 1000;
                theServo.setPosition( crrPos );
            }
        });
        valueAnimator.start();
        while( valueAnimator.isRunning() ){

        }
        theServo.setPosition( Range.clip( getAdjustedPositionSafe( pos ), 0.0, 1.0 ) );
 */

    double getAdjustedPositionSafe ( double pos ) {

        pos = Range.clip( pos, 0.0, 1.0 );

        double pos_255 = A0_255 + (A180_255 - A0_255) * pos;

        pos_255 = Range.clip( pos_255, minLimit_255, maxLimit_255 );

        double pos_1 = pos_255 / ( 255 );

        return Range.clip( pos_1, 0.0, 1.0 );
    }
}
