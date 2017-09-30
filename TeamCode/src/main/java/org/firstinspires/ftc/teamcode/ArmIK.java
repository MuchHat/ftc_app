package org.firstinspires.ftc.teamcode;

/**
 * Created by gigel on 2017-09-29.
 */

public class ArmIK {

    Integer[] xArray = null;
    Integer[] yArray = null;
    Integer[] zArray = null;

    Arm testArm = null;

    public void init() {

        xArray = new Integer[ 1000 ];
        yArray = new Integer[ 1000 ];
        zArray = new Integer[ 1000 ];

        testArm = new Arm();

        testArm.init();

        for (int t = 0; t < 10; t++) {
            for (int b = 0; b < 10; b++) {
                for (int e = 0; e < 10; e++) {
                    testArm.setServos( (double) t/ 10, (double) b / 10, (double) e / 10);
                        int index = t + b * 10 + e * 100;
                        xArray[index] = (int) testArm.getX();
                        yArray[index] = (int) testArm.getY();
                        zArray[index] = (int) testArm.getZ();
                    }

                }

            }
    }
}
