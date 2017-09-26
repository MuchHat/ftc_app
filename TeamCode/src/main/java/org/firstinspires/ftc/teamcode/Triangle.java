package org.firstinspires.ftc.teamcode;

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

    public void resolve_SAA( double al1, double aa2, double aa3 ){

        l1 = al1;

        a2 = aa2;
        a3 = aa3;
        a1 = Math.PI - a2 - a3;

        l2 = l1 * Math.sin( a2 ) / Math.sin( a1 );
        l3 = l1 * Math.sin( a3 ) / Math.sin( a1 );

        isValid = true;
    }

    public void resolve_SSA( double al1, double al2, double aa3 ) {

        l1 = al1;
        l2 = al2;

        a3 = aa3;

        l3 = Math.sqrt(l2 * l2 + l1 * l1 - 2 * l2 * l1 * Math.cos(a3));

        if (l1 > l2) {
            a2 = Math.asin( l2 * Math.sin( a3 ) / l3 );
            a1 = Math.PI - a2 - a3;
        }
        if (l2 > l1) {
            a1 = Math.asin( l1 * Math.sin( a3 ) / l3 );
            a2 = Math.PI - a1 - a3;
        }

        isValid = true;
    }

    public void resolve_SSS( double al1, double al2, double al3 ){

        l1 = al1;
        l2 = al2;
        l3 = al3;

        a1 = Math.acos( ( l2 * l2 + l3 * l3 - l1 * l1 ) / ( 2 * l2 * l3 ) );
        a2 = Math.acos( ( l1 * l1 + l3 * l3 - l2 * l2 ) / ( 2 * l2 * l3 ) );
        a3 = Math.PI - a1 - a2;

        isValid = true;
    }
}
