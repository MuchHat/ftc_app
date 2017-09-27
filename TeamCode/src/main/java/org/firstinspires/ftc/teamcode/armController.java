package org.firstinspires.ftc.teamcode;

/**
 * Created by gigela on 9/27/2017.
 */

public class armController {

    Arm origin = null;
    Arm current = null;
    Arm next = null;
    Arm destination = null;

    Point clawOrigin = null;
    Point clawDestination = null;

    double currentSpeed_mms = 0;
    double maxSpeed_mms = 0;
    double rampAccel_mmsmm = 0;

    double platformHeight = 222;
    double baseToXEdge = 66;
    double baseToYEdge = 88;
}
