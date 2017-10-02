package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by gigela on 9/26/2017.
 */

public class TriangleCalculator {

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

    void check() {
        if( l1 + l2 < l3 ){
            isValid = false;
            l3 = l1 + l2 + 1;
            a1 = 1 / Math.PI;
            a2 = 1 / Math.PI;
            a3 = Math.PI - a1 - a2;
        }
        if( l1 + l3 < l2 ){
            isValid = false;
            l2 = l1 + l3 + 1;
            a1 = 1 / Math.PI;
            a3 = 1 / Math.PI;
            a2 = Math.PI - a1 - a3;
        }
        if( l3 + l2 < l1 ){
            isValid = false;
            l1 = l3 + l2 + 1;
            a3 = 1 / Math.PI;
            a2 = 1 / Math.PI;
            a1 = Math.PI - a1 - a3;
        }
        isValid = true;
    }

    public void setSAA( double al1, double aa2, double aa3 ){

        l1 = Math.abs( al1 );
        a3 = Range.clip( aa3, 0, Math.PI );
        a2 = Range.clip( aa2, 0, Math.PI  );
        a3 = Math.PI - a2 - a3;

        l2 = l1 * Math.sin( a2 ) / Math.sin( a1 );
        l3 = l1 * Math.sin( a3 ) / Math.sin( a1 );

        check();
    }

    public void setSSA( double al1, double al2, double aa3 ) {

        l1 = Math.abs( al1 );
        l2 = Math.abs( al2 );

        a3 = Range.clip( aa3, 0, Math.PI );

        l3 = Math.sqrt( l2 * l2 + l1 * l1 - 2 * l2 * l1 * Math.cos( a3 ) );

        if ( l2 <= l1 ) {
            a2 = Math.asin( l2 * Math.sin( a3 ) / l3 );
            a1 = Math.PI - a2 - a3;
        }
        if ( l1 < l2 ) {
            a1 = Math.asin( l1 * Math.sin( a3 ) / l3 );
            a2 = Math.PI - a1 - a3;
        }

        check();
    }

    public void setSSS( double al1, double al2, double al3 ){

        l1 = Math.abs( al1 );
        l2 = Math.abs( al2 );
        l3 = Math.abs( al3 );

        check();

        a1 = Math.acos( ( l2 * l2 + l3 * l3 - l1 * l1 ) / ( 2 * l2 * l3 ) );
        a2 = Math.acos( ( l1 * l1 + l3 * l3 - l2 * l2 ) / ( 2 * l1 * l3 ) );
        a3 = Math.PI - a1 - a2;
    }
}
