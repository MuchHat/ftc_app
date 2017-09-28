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

        altitude  = Range.clip( Math.abs( altitude ), 0, Math.PI );
        azimuth = Range.clip( Math.abs( azimuth ), 0, Math.PI );

        if( altitude > Math.PI /2 ) altitude = -( altitude - Math.PI / 2);
        if( azimuth > Math.PI /2 ) azimuth = -( azimuth - Math.PI / 2);

        isValid = true;
    }
    public void solve_RAA( double sr, double aazimuth, double aaltitude ) {

        r = sr;

        altitude = Range.clip( aaltitude, -Math.PI / 2, Math.PI / 2 );
        azimuth = Range.clip( aazimuth, -Math.PI / 2, Math.PI / 2 );

        double altitude_0_pi = altitude;
        if( altitude_0_pi < 0 ) altitude_0_pi = Math.PI - altitude_0_pi;
        double azimut_0_pi = azimuth;
        if( azimut_0_pi < 0 ) azimut_0_pi = Math.PI - azimut_0_pi;

        x = Math.cos( altitude_0_pi ) * Math.cos( azimut_0_pi );
        y = Math.cos( altitude_0_pi ) * Math.sin( azimut_0_pi );

        isValid = true;
    }

    public double distanceTo( Point to ){

        return Math.sqrt( ( ( to.x - x  ) * ( to.x - x  ) ) +
                ( ( to.y - y ) * ( to.y - y ) ) +
                ( ( to.z - z ) * ( to.z - z ) )
        );
    }
}
