package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by gigela on 9/27/2017.
 */

public class Angle {

    double angleServo = 0;
    double anglePI = 0;

    double offsetServo = 0;  // servo 0.2 conicides with 0 PI
    double slopeServo = 1.0; // 1 PI is 0.8 of the servo
    double minServo = 0.05;
    double maxServo = 0.95;

    public void solve_AngleServo(double as) {

        anglePI = ((angleServo - offsetServo) / slopeServo) * Math.PI;

        anglePI = Range.clip( anglePI, 0, Math.PI );
        anglePI -= Math.PI/2;
    }

    public void solve_AnglePI(double ap) {

        anglePI = Range.clip( ap, -Math.PI/2, Math.PI/2 );

        angleServo = (anglePI / Math.PI) * slopeServo + offsetServo;
        angleServo += 0.5;

        Range.clip( angleServo, minServo, maxServo );
    }
}

