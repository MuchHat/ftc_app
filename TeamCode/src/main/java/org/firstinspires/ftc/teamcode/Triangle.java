package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by gigela on 9/26/2017.
 */

public class Triangle {

    public double a1 = 0;
    public double a2 = 0;
    public double a3 = 0;

    public double l1 = 0;
    public double l2 = 0;
    public double l3 = 0;

    public boolean isValid = false;

    /* Constructor */
    public void Triangle(){
    }

    public void solve_SAA( double al1, double aa2, double aa3 ){

        l1 = Math.abs( al1 );

        a3 = aa3;
        a3 = Range.clip( a3, -Math.PI / 2, Math.PI / 2 );
        a2 = aa3;
        a2 = Range.clip( a2, -Math.PI / 2, Math.PI / 2 );

        double a3_0_pi = a3;
        if( a3_0_pi < 0 ) a3_0_pi = Math.PI / 2 - a3_0_pi;
        double a2_0_pi = a2;
        if( a2_0_pi < 0 ) a2_0_pi = Math.PI / 2 - a2_0_pi;
        double a1_0_pi = Math.PI - a2_0_pi - a3_0_pi;
        a1_0_pi = a1;
        if( a1_0_pi < 0 ) a1_0_pi = Math.PI / 2 - a1_0_pi;

        l2 = l1 * Math.sin( a2_0_pi ) / Math.sin( a1_0_pi );
        l3 = l1 * Math.sin( a3_0_pi ) / Math.sin( a1_0_pi );

        a1 = a1_0_pi;
        if( a1 > Math.PI / 2 ) a1 = -( a1 - Math.PI / 2 );
        a2 = a2_0_pi;
        if( a2 > Math.PI / 2 ) a2 = -( a2 - Math.PI / 2 );
        a3 = a3_0_pi;
        if( a3 > Math.PI / 2 ) a3 = -( a3 - Math.PI / 2 );

        isValid = true;
    }

    public void solve_SSA( double al1, double al2, double aa3 ) {

        l1 = Math.abs( al1 );
        l2 = Math.abs( al2 );

        a3 = aa3;
        a3 = Range.clip( a3, -Math.PI / 2, Math.PI / 2 );

        double a3_0_pi = a3;
        if( a3_0_pi < 0 ) a3_0_pi = Math.PI / 2 - a3_0_pi;
        double a2_0_pi = a2;
        if( a2_0_pi < 0 ) a2_0_pi = Math.PI / 2 - a2_0_pi;
        double a1_0_pi = a2;
        if( a1_0_pi < 0 ) a1_0_pi = Math.PI / 2 - a1_0_pi;

        l3 = Math.sqrt( l2 * l2 + l1 * l1 - 2 * l2 * l1 * Math.cos( a3_0_pi ) );

        if ( l1 > l2 ) {
            a2_0_pi = Math.asin( l2 * Math.sin( a3_0_pi ) / l3 );
            a1_0_pi = Math.PI - a2_0_pi - a3_0_pi;
        }
        if ( l2 > l1 ) {
            a1_0_pi = Math.asin( l1 * Math.sin( a3_0_pi ) / l3 );
            a2_0_pi = Math.PI - a1_0_pi - a3_0_pi;
        }

        a1 = a1_0_pi;
        if( a1 > Math.PI / 2 ) a1 = -( a1 - Math.PI / 2 );
        a2 = a2_0_pi;
        if( a2 > Math.PI / 2 ) a2 = -( a2 - Math.PI / 2 );
        a3 = a3_0_pi;
        if( a3 > Math.PI / 2 ) a3 = -( a3 - Math.PI / 2 );

        isValid = true;
    }

    public void solve_SSS( double al1, double al2, double al3 ){

        l1 = Math.abs( al1 );
        l2 = Math.abs( al2 );
        l3 = Math.abs( al3 );

        if( ( l1 + l2 < l3 ) ||
                ( l1 + l3 < l2 ) ||
                ( l3 + l2 < l1 ) ) {
            a1 = 0;
            a2 = 0;
            a3 = 0;
            isValid = false;
            return;
        }

        a1 = Math.acos( ( l2 * l2 + l3 * l3 - l1 * l1 ) / ( 2 * l2 * l3 ) );
        a2 = Math.acos( ( l1 * l1 + l3 * l3 - l2 * l2 ) / ( 2 * l2 * l3 ) );
        a3 = Math.PI - a1 - a2;

        if( a1 > Math.PI /2 ) a1 = -( a1 - Math.PI / 2 );
        if( a2 > Math.PI /2 ) a2 = -( a2 - Math.PI / 2 );
        if( a3 > Math.PI /2 ) a3 = -( a3 - Math.PI / 2 );

        isValid = true;
    }
}
