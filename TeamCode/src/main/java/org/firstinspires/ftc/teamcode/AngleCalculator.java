package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by gigela on 9/27/2017.
 */

public class AngleCalculator {

    private double angleServo = 0; // between 0 and 1.0
    private double anglePI = 0;  // between -pi/2 to pi/2

    private double offsetServo = 0;  // servo 0.2 coincides with 0 PI
    private double slopeServo = 1.0; // 1 PI is 0.8 of the servo
    private double minServo = 0.05;
    private double maxServo = 0.95;

    public void AngleCalculator(){

    }

    public void init( double s45, double s135, double aMin, double aMax ) {

        slopeServo = 0.5 / ( s135 - s45 );
        offsetServo = 0.25 - ( s45 * slopeServo );
        minServo = aMin;
        maxServo = aMax;
    }

    public double getServo( double aAnglePI ){

        setPI( aAnglePI );

        return angleServo; }

    public double getPI( double aAngleServo){

        setServo( aAngleServo );

        return anglePI; }

    private void setServo( double as ) {

        angleServo = Range.clip( as, minServo, maxServo );

        anglePI = ( angleServo * slopeServo + offsetServo ) * Math.PI;

        anglePI = Range.clip( anglePI, 0, Math.PI );
    }

    private void setPI( double ap ) {

        anglePI = Range.clip( ap, 0, Math.PI );

        angleServo = ( anglePI / Math.PI  - offsetServo ) / slopeServo;

        angleServo = Range.clip( angleServo, minServo, maxServo );

    }
}

