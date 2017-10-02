package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by MuchHat on 9/27/2017.
 */

public class Angle {

    public double angleServo = 0; // between 0 and 1.0
    public double anglePI = 0;  // between -pi/2 to pi/2

    public double offsetServo = 0;  // servo 0.2 coincides with 0 PI
    public double slopeServo = 1.0; // 1 PI is 0.8 of the servo
    public double minServo = 0.05;
    public double maxServo = 0.95;

    public void Angle() {

    }

    public void Init_45_135( double s45, double s135, double aMin, double aMax ) {

        slopeServo = 0.5 / ( s135 - s45 );
        offsetServo = 0.25 - ( s45 * slopeServo );
        minServo = aMin;
        maxServo = aMax;
    }

    public double getServo(){ return angleServo; }

    public double getPI(){ return anglePI; }

    public void setServo( double as ) {

        angleServo = Range.clip( as, minServo, maxServo );

        anglePI = ( angleServo * slopeServo + offsetServo ) * Math.PI;

        anglePI = Range.clip( anglePI, 0, Math.PI );

        //Log.d( "MuchHat", String.format( "Angle_setServo: %.3f, %.3f pi", angleServo, anglePI ) );
    }

    public void setPI( double ap ) {

        anglePI = Range.clip( ap, 0, Math.PI );

        angleServo = ( anglePI / Math.PI  - offsetServo ) / slopeServo;

        angleServo = Range.clip( angleServo, minServo, maxServo );

        //Log.d( "MuchHat", String.format( "Angle_setPI: %.3f, %.3fpi", angleServo, anglePI ) );

    }
}

