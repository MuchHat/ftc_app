package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by MuchHat on 9/27/2017.
 */

public class Angle {

    public double angleServo = 1.0; // between 0 and 1.0
    public double anglePI = 0;  // between -pi/2 to pi/2

    public double offsetServo = 0;  // servo 0.2 coincides with 0 PI
    public double slopeServo = 1.0; // 1 PI is 0.8 of the servo
    public double minServo = 0.05;
    public double maxServo = 0.95;

    public void Angle() {

    }

    public void Init( double angle1, double angle2, double pos1, double pos2, double aMin, double aMax ) {

        // angle1 and angle2 are the observed angles for pos1 and pos2 -
        // angle1 and angle2 are in 0.0->1.1

        slopeServo = ( angle2 - angle1 ) / ( pos2 - pos1 );
        offsetServo = angle1 - ( pos1 * slopeServo );
        minServo = aMin;
        maxServo = aMax;
    }

    public double getServo(){ return angleServo; }

    public double getPI(){ return anglePI; }

    public void setServo( double as ) {

        angleServo = Range.clip( as, minServo, maxServo );

        anglePI = ( angleServo * slopeServo + offsetServo ) * Math.PI;

        anglePI = Range.clip( anglePI, 0, Math.PI );

    }

    public void setPI( double ap ) {

        anglePI = Range.clip( ap, 0, Math.PI );

        angleServo = ( anglePI / Math.PI  - offsetServo ) / slopeServo;

        angleServo = Range.clip( angleServo, minServo, maxServo );

    }
}

