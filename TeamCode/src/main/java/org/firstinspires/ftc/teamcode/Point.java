package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.util.Range.*;

/**
 * Created by gigel on 2017-09-27.
 */

public class Point {

    public double azimuth = 0; // -pi/2 to pi/2
    public double altitude = 0; // -pi/2 to pi/2

    public double x = 0;
    public double y = 0;
    public double z = 0;

    public double r = 0;

    public boolean isValid = false;

    public void Point(){
    }

    public void solve_XYZ( double x, double y, double z ) {


        r = Math.sqrt( x * x + y * y + z * z );

        double sign_x = x > 0 ? 1.0 : -1.0;
        double sign_y = y > 0 ? 1.0 : -1.0;
        double sign_z = z > 0 ? 1.0 : -1.0;

        altitude = Math.asin( z / r * sign_z ) * sign_z;
        azimuth = Math.atan( y / x * sign_x * sign_y ) * sign_x * sign_y;

        isValid = true;
    }
    public void solve_RAA( double sr, double aazimuth, double aaltitude ) {

        r = sr;

        altitude = Range.clip( aaltitude, -Math.PI / 2, Math.PI / 2 );
        azimuth = Range.clip( aazimuth, -Math.PI / 2, Math.PI / 2 );

        double sign_altitude = altitude > 0 ? 1.0 : -1.0;
        double sign_azimuth = azimuth > 0 ? 1.0 : -1.0;

        x = Math.cos( altitude * sign_altitude ) * Math.cos( azimuth * sign_azimuth ) * sign_altitude * sign_azimuth;
        y = Math.cos( altitude * sign_altitude ) * Math.sin( azimuth* sign_altitude ) * sign_altitude * sign_azimuth;

        isValid = true;
    }

    public double distanceTo( Point to ){

        return Math.sqrt( ( ( to.x - x  ) * ( to.x - x  ) ) +
                ( ( to.y - y ) * ( to.y - y ) ) +
                ( ( to.z - z ) * ( to.z - z ) )
        );
    }
}
