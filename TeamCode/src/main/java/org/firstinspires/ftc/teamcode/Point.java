package org.firstinspires.ftc.teamcode;

/**
 * Created by gigel on 2017-09-27.
 */

public class Point {

    public double ax = 0;
    public double az = 0;

    public double x = 0;
    public double y = 0;
    public double z = 0;

    public double r = 0;

    public Triangle xProjection = new Triangle();
    public Triangle zProjection = new Triangle();

    public boolean isValid = false;

    public void Point(){
    }

    public void solve_XYZ( double x, double y, double z ) {

        Triangle t = new Triangle();

        // for z
        t.solve_SSA( x, y, Math.PI / 2 );

        zProjection.solve_SSA( t.l3, z, Math.PI / 2 );
        az = zProjection.a2;

        // for x
        t.solve_SSA( y, z, Math.PI / 2 );

        xProjection.solve_SSA( t.l3, x, Math.PI / 2 );
        r = xProjection.l3;
        ax = xProjection.a2;

        isValid = true;
    }
    public void solve_R_AZ_AX( double sr, double az, double ax ) {

        Triangle t = new Triangle();

        r = sr;

        // for z
        t.solve_SAA( r, az, Math.PI / 2 - az );
        zProjection.solve_SSA( t.l3, t.l2, t.a2 );
        z = zProjection.l2;

        // for x
        t.solve_SAA( r, ax, Math.PI / 2 - ax );
        xProjection.solve_SSA( t.l3, t.l2, t.a2 );

        x = xProjection.l2;

        y = xProjection.l3;

        isValid = true;
    }

    public double distanceTo( Point to ){

        Point p = new Point();

        p.solve_XYZ( to.x - x, to.y - y, to.z - z );

        return p.r;
    }
}
