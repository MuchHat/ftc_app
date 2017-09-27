package org.firstinspires.ftc.teamcode;

/**
 * Created by gigela on 9/27/2017.
 */

public class Angle {

    double angleServo = 0;
    double anglePI = 0;

    double offsetServo = 0;  // e.g. add servo 0 starts at 0.2
    double slopeServo = 1.0; // 1 PI is 0.8 of the servo
    double minServo = 0.05;
    double maxServo = 0.95;

    public void solve_AngleServo( double as ) {

        angleServo = as;
        anglePI = ( ( angleServo + offsetServo ) / slopeServo ) * Math.PI - Math.PI / 2;

        check();
    }

    public void solve_AnglePI ( double ap ) {

        anglePI = ap;
        angleServo = ( ( anglePI  + Math.PI / 2 ) / Math.PI ) * slopeServo - offsetServo;

        check();
    }

    public void check() {

        if( angleServo > maxServo )
            angleServo = maxServo;
        if( angleServo < minServo )
            angleServo = minServo;

        solve_AngleServo( angleServo );
    }
}
