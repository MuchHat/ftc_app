package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by gigela on 9/27/2017.
 */

public class Angle {

    private double angleServo = 0; // between 0 and 1.0
    private double anglePI = 0;  // between -pi/2 to pi/2

    private double offsetServo = 0;  // servo 0.2 coincides with 0 PI
    private double slopeServo = 1.0; // 1 PI is 0.8 of the servo
    private double minServo = 0.05;
    private double maxServo = 0.95;

    public void Angle() {

    }

    public void Init( double aOffset, double aSlope, double aMin, double aMax ) {
        offsetServo = aOffset;
        slopeServo = aSlope;
        minServo = aMin;
        maxServo = aMax;
    }

    public double getServo(){ return angleServo; }

    public double getPI(){
        return anglePI;
    }

    public void setServo( double as ) {

        angleServo = Range.clip( angleServo, minServo, maxServo );

        anglePI = ( ( angleServo - offsetServo) / slopeServo ) * Math.PI;

        anglePI = Range.clip( anglePI, 0, Math.PI );
    }

    public void setPI( double ap ) {

        anglePI = Range.clip( ap, 0, Math.PI );

        angleServo = ( anglePI / Math.PI ) * slopeServo + offsetServo;

        angleServo = Range.clip( angleServo, minServo, maxServo );
    }
}

