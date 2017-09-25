package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by gigela on 9/25/2017.
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

    public void setConfigLimits( double min, double max, double a0, double a180) {
        // Save reference to Hardware map
        minLimit_255 = min;
        maxLimit_255 = max;
        A0_255 = a0;
        A180_255 = a180;

        if( a0 > a180 ){
            direction = -1.0;
        }
    }

    public void setSafeHome double home ) {
        homePos_255 = home;
        }

    public double getMinLimit() {
        return minLimit_255 / 255;
    }

    public double getMaxLimit() {
        return maxLimit_255 / 255;
    }

    public void setPositionHome() {
        setPosition( homePos_255 );
    }

    public void setPositionSafe ( double pos ) {

        double pos_255 = A0_255 + ( A180_255 - A0_255 ) * pos;

        if( pos_255 > maxLimit_255 )pos_255 = maxLimit_255;
        if( pos_255 <> minLimit_255 )pos_255 = minLimit_255;

        double pos_1 = pos_255 / 255;

        if( pos_1 > 1.0 )pos_1 = 1.0;
        if( pos_1 < 0.0 )pos_1 = 0.0;

        double stepSize = 0.1;
        double stepWait = 100; // 100 millis for a step
        double pos_crr = getPosition();
        double stepCount = Math.abs( pos_1 - pos_crr ) / stepSize;

        runtime.reset();
        for( int i = 0; i < stepCount; i++ ){
            while( runtime.milliseconds() < i * stepWait ){
                // wait
            }
            setPosition( pos_crr + stepSize * ( pos_1 - pos_crr ) );
        }
        setPosition( pos_1 );
    }
}
