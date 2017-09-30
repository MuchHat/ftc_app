package org.firstinspires.ftc.teamcode;

/**
 * Created by gigel on 2017-09-29.
 */

public class ArmIK {

    Integer[] xArray = null;
    Integer[] yArray = null;
    Integer[] zArray = null;

    public double b = 0;
    public double t = 0;
    public double e = 0;

    public double x = 0;
    public double y = 0;
    public double z = 0;

    Arm testArm = null;

    public void init() {

        xArray = new Integer[ 27000 ];
        yArray = new Integer[ 27000 ];
        zArray = new Integer[ 27000 ];

        testArm = new Arm();

        testArm.init();

        for (int t = 0; t < 30; t++) {
            for (int b = 0; b < 30; b++) {
                for (int e = 0; e < 30; e++) {
                    testArm.setServos( (double) t/ 30, (double) b / 30, (double) e / 30);
                        int index = t + b * 30 + e * 30 * 30;
                        xArray[ index ] = (int) testArm.getX();
                        yArray[ index ] = (int) testArm.getY();
                        zArray[ index ] = (int) testArm.getZ();
                    }

                }

            }
    }
     public boolean solveXYZ( double ax, double ay, double az )
     {
         double dMax = 99999;
         double index = -1;

         for (int i = 0; i < 27000; i++) {
             double cx = xArray[i];
             double cy = yArray[i];
             double cz = zArray[i];

             double d = Math.sqrt((cx - ax) * (cx - ax) +
                     (cy - ay) * (cy - ay) +
                     (cz - az) * (cz - az));
             if (d < dMax) {
                 dMax = d;
                 index = i;
             }
         }

         if( index < 0 ) {
             return false;
         }

         e = index / ( 30 * 30 );
         index -= e * 30 * 30;

         b = index / 30;
         index -= e * 30;

         t = index;

         return true;

         }

     }

