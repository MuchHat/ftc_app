package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by MuchHat on 9/25/2017.
 */

public abstract class SafeServo extends Servo {

    /* local OpMode members. */
    private ElapsedTime runtime  = new ElapsedTime();

    private double A0_255 = 0;
    private double A180_255 = 255;
    private double minLimit_255 = 0;
    private double maxLimit_255 = 255;
    private double homePos_255 = 128;
    private double direction = 1.0;

    /* Constructor */
    public SafeServo(){

    }

    public void setConfigLimits( double min_255, double max_255, double a0_255, double a180_255) {
        // Save reference to Hardware map
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

    public void setConfigHome ( double home ) {
        homePos_255 = Range.clip( home, minLimit_255, maxLimit_255 );
        }

    public double getMinLimit() {
        return Range.clip( ( minLimit_255 - A0_255 ) / ( A180_255 - A0_255 ), 0.0, 1.0 );
    }

    public double getMaxLimit() {
        return Range.clip( ( maxLimit_255 - A0_255 ) / ( A180_255 - A0_255 ), 0.0, 1.0 );
    }

    public void setPositionSafeHome() {

        double pos_1 = ( homePos_255 - A0_255 ) /  ( A180_255 - A0_255 );
        setPositionSafeSmooth( pos_1 );
    }

    public void setPositionSafe ( double pos ) {

        setPosition( getAdjustedPositionSafe( pos ) );
    }

    public void setPositionSafeSmooth ( double pos ) {

        double pos_1 = getAdjustedPositionSafe( pos );
        double pos_crr = getPosition();

        double stepSize  = 0.1;
        double stepWait  = 100; // 100 millis for a step
        double rampRatio = 0.2; // increase/decrease time per step during ramp

        double stepCount = Math.abs( pos_1 - pos_crr ) / stepSize;
        double rampSteps = stepCount * 0.2; // 20% ramp up/down
        if( rampSteps < 2 )rampSteps = 2;
        if( rampSteps > stepCount / 2 ) rampSteps = stepCount / 2;

        runtime.reset();
        double waitTarget = 0;
        for( int i = 0; i < stepCount; i++ ){

            waitTarget += stepWait;

            double rampPosition = Math.max( rampSteps - i, i - ( stepCount - rampSteps )  );
            rampPosition = Math.max( rampPosition, 0 );
            rampPosition = Math.min( rampPosition, rampSteps );

            waitTarget += stepWait * rampRatio * rampPosition;

            setPosition( pos_crr + stepSize * ( pos_1 - pos_crr ) );

            while( runtime.milliseconds() < waitTarget ){
                // wait
            }
        }
        setPosition( Range.clip( pos_1, 0.0, 1.0 ) );
    }

    double getAdjustedPositionSafe ( double pos ) {

        pos = Range.clip( pos, 0.0, 1.0 );

        double pos_255 = A0_255 + (A180_255 - A0_255) * pos;

        pos_255 = Range.clip( pos_255, minLimit_255, maxLimit_255 );

        double pos_1 = pos_255 / ( 255 );

        return Range.clip( pos_1, 0.0, 1.0 );
    }



}
