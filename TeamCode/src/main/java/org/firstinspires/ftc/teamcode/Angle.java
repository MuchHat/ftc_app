package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by gigela on 9/27/2017.
 */

public class Angle {

    double angleServo = 0; // between 0 and 1.0
    double anglePI = 0;  // between -pi/2 to pi/2

    double offsetServo = 0;  // servo 0.2 coincides with 0 PI
    double slopeServo = 1.0; // 1 PI is 0.8 of the servo
    double minServo = 0.05;
    double maxServo = 0.95;

    public void Angle() {

    }

    // all PI angles are 0 to PI

    public void solve_angleServo( double as ) {

        angleServo = Range.clip( angleServo, minServo, maxServo );

        anglePI = ( ( angleServo - offsetServo) / slopeServo ) * Math.PI;

        anglePI = Range.clip( anglePI, 0, Math.PI );
    }

    public void solve_anglePI( double ap ) {

        anglePI = Range.clip( ap, 0, Math.PI );

        angleServo = ( anglePI / Math.PI ) * slopeServo + offsetServo;

        angleServo = Range.clip( angleServo, minServo, maxServo );
    }
}

