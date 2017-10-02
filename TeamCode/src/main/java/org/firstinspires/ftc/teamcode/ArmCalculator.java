package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by gigela on 10/1/2017.
 */

public class ArmCalculator {

    private double _lBase = 288;
    private double _lElbow = 188;

    private double _xMin = 11;
    private double _yMin = 11;
    private double _zMin = -111;
    private double _xMax = 333;
    private double _yMax = 333;
    private double _zMax = 333;
    private double _rMin = 22;
    private double _rMax = ( _lBase + _lElbow ) * 0.95;

    private double _r = 0;
    private double _phi = 0;
    private double _teta = 0;
    private double _a2 = 0;

    private double _x = 0;
    private double _y = 0;
    private double _z = 0;

    private double _turret = 0;
    private double _base = 0;
    private double _elbow = 0;
    private double _wrist = 0;

    private void setXYZ( double ax, double ay, double az )
    {
        _x = Range.clip( ax, _xMin, _xMax );
        _y = Range.clip( ay, _yMin, _yMax );
        _z = Range.clip( az, _zMin, _zMax );

        _r = Range.clip( Math.sqrt( _x * _x + _y * _y + _z * _z ), _rMin, _rMax );

        _phi = Math.acos( _z / _r );

        if( Math.abs( _x ) <= 1 ){
            _teta = Math.PI / 2;
        }
        else if( Math.abs( _x ) > 1 ){
            _teta = Math.atan( Math.abs( _y / _x ) );
            if( _x < 0 )_teta += Math.PI / 2;
        }

        TriangleCalculator elbowTriangle = new TriangleCalculator();
        elbowTriangle.setSSS( _lBase, _lElbow, _r );

        _a2 = elbowTriangle.a2;

        _turret = Math.PI - _teta;
        _base = Math.PI - _phi + elbowTriangle.a2 ;
        _elbow = elbowTriangle.a3;
        _wrist = Math.PI /2 - elbowTriangle.a1 - _phi;
    }

    public double getBase(double x, double y, double z) {

        setXYZ( x, y, z );
        return _base;
    }

    public double getElbow(double x, double y, double z) {

        setXYZ( x, y, z );
        return _elbow;
    }

    public double getWrist(double x, double y, double z) {

        setXYZ( x, y, z );
        return _wrist;
    }

    public double getTurret(double x, double y, double z) {

        setXYZ( x, y, z );
        return _turret;
    }
}

