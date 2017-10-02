package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Created by gigela on 9/27/2017.
 */

public class Arm {

    public Angle turretAngle = null;
    public Angle baseAngle = null;
    public Angle elbowAngle = null;
    public Angle wristAngle = null;
    public Angle wristHorizontalAngle = null;
    public Angle rightClawAngle = null;
    public Angle leftClawAngle = null;
    double mmClawOpen = 222;  // mm
    double mmClawClose = 188; // mm
    double lBase = 188; // mm
    double lElbow = 233; // mm
    double lClawArm = 150; // mm
    double lClawGap = 37;  // mm
    //TODO
    double xMax = 333; // mm
    double xMin = -333; // mm
    double yMax = 333; // mm
    double yMin = 0; // mm
    double zMax = 333; // mm
    double zMin = -111; // mm
    double rMax = (lBase + lElbow) * 0.95; // mm
    double rMin = 11;
    double armBaseLocationX = 444;
    double armBaseLocationY = 222;
    double armBaseLocationZ = 11;
    double robotHeight = 111;
    double robotWidth = 111;
    double robotLenght = 333;
    boolean isInitialized = false;
    private double x = 0;
    private double y = 0;
    private double z = 0;
    private double teta = 0;
    private double phi = 0;
    private double r = 0;
    // TODO END
    private double e_a2 = 0;
    private double clawMM = 0;

    /* Constructor */
    public void Arm() {
    }

    public void init() {

        turretAngle = new Angle();
        baseAngle = new Angle();
        elbowAngle = new Angle();

        wristAngle = new Angle();
        wristHorizontalAngle = new Angle();
        rightClawAngle = new Angle();
        leftClawAngle = new Angle();

        turretAngle.Init_45_135(0.140, 0.644, 0.05, 0.897); // turret setup
        baseAngle.Init_45_135(0.25, 0.75, 0.05, 0.95); // TODO
        elbowAngle.Init_45_135(0.404, 0.950, 0.20, 0.95); //elbow setup
        wristAngle.Init_45_135(1.1775, 0.6105, 0.32, 0.89);  //wrist setup
        rightClawAngle.Init_45_135(0.221, -0.437, 0.221, 0.55); // right claw setup
        leftClawAngle.Init_45_135(0.818, 1.582, 0.436, 0.818); // left claw setup

        setClawMM(lClawGap);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public void setHomeXYZ() {
        setXYZ(0, armBaseLocationY, robotHeight);
    }

    public void setFrontXYZ() {
        setXYZ(0, armBaseLocationY + 33, zMin);
    }

    public void setZeroXYZ() {
        setXYZ(0, yMin, robotHeight);
    }

    public double getR() {
        return r;
    }

    public double getTeta() {
        return teta;
    }

    public double getPhi() {
        return phi;
    }

    public double getA2() {
        return e_a2;
    }

    public double getBaseServo() {
        return baseAngle.getServo();
    }

    public double getTurretServo() {
        return turretAngle.getServo();
    }

    public double getElbowServo() {
        return elbowAngle.getServo();
    }

    public double getWristServo() {
        return wristAngle.getServo();
    }

    public double getRightClawServo() {
        return rightClawAngle.getServo();
    }

    public double getLeftClawServo() {
        return leftClawAngle.getServo();
    }

    public void copyFrom(Arm anotherArm) {
        setXYZ(anotherArm.getX(), anotherArm.getY(), anotherArm.getZ());
        setClawMM(anotherArm.getClawMM());
    }

    public double getClawMM() {
        return clawMM;
    }

    public void setClawMM(double amm) {

        Triangle clawTriangle = new Triangle();

        amm = Math.abs(amm);

        double l1 = (amm - lClawGap) / 2;
        double l3 = lClawArm;

        if (l3 < l1 * 0.85) l3 = l1 * 0.88; // such it does not open too much

        double l2 = Math.sqrt(l3 * l3 - l1 * l1);

        clawTriangle.setSSS(l1, l2, l3);
        rightClawAngle.setPI(clawTriangle.a1);
        leftClawAngle.setPI(clawTriangle.a1);

        clawMM = amm;
    }

    public double distanceTo(Arm anotherArm) {

        return Math.sqrt((x - anotherArm.getX()) * (x - anotherArm.getX()) +
                (y - anotherArm.getY()) * (y - anotherArm.getY()) +
                (z - anotherArm.getZ()) * (z - anotherArm.getZ()));

    }

    public void setServos(double ts, double bs, double es) {

        turretAngle.setServo(ts);
        baseAngle.setServo(bs);
        elbowAngle.setServo(es);

        teta = Math.PI - turretAngle.getPI();

        Triangle elbowTriangle = new Triangle();
        elbowTriangle.setSSA(lBase, lElbow, elbowAngle.getPI());

        e_a2 = elbowTriangle.a2;

        phi = baseAngle.getPI() - Math.PI / 2 + elbowTriangle.a2;

        r = elbowTriangle.l3;

        z = Range.clip(r * Math.cos(phi), zMin, zMax); // works > pi/2 and < 0
        x = Range.clip(r * Math.sin(phi) * Math.cos(teta), xMin, xMax);
        y = Range.clip(r * Math.sin(phi) * Math.sin(teta), yMin, yMax);

    }

    public void setXYZ(double ax, double ay, double az) {

        x = Range.clip(ax, xMin, xMax);
        y = Range.clip(ay, yMin, yMax);
        z = Range.clip(az, zMin, zMax);

        colisionCheck(true);

        r = Range.clip(Math.sqrt(x * x + y * y + z * z), rMin, rMax);

        phi = Math.acos(z / r);

        if (Math.abs(x) <= 0.001) {
            teta = Math.PI / 2;
        } else if (Math.abs(x) > 0.001) {
            teta = Math.atan(Math.abs(y / x));
            if (x < 0) teta += Math.PI / 2;
        }

        Triangle elbowTriangle = new Triangle();
        elbowTriangle.setSSS(lBase, lElbow, r);

        e_a2 = elbowTriangle.a2;

        turretAngle.setPI(Math.PI - teta);
        baseAngle.setPI(Math.PI - phi + elbowTriangle.a2);
        elbowAngle.setPI(elbowTriangle.a3);
    }

    public boolean colisionCheck(boolean adjust) {

        boolean collisionDetected = false;

        // check if hitting the ground
        if (z < -robotHeight) {
            collisionDetected = true;
            if (adjust) z = armBaseLocationZ;
        }

        // check if hitting the robot
        if ((x > -armBaseLocationX && x < armBaseLocationX) &&
                (y < armBaseLocationY) &&
                (z < armBaseLocationZ)) {
            collisionDetected = true;
            if (adjust) {
                // move to the closest
                double top = armBaseLocationZ - z;
                double front = armBaseLocationY - y;
                double left = armBaseLocationX + x;
                double right = armBaseLocationX - x;

                double min = Math.min(top, front);
                min = Math.min(min, left);
                min = Math.min(min, right);

                if (left == min) x = armBaseLocationX;
                if (right == min) x = armBaseLocationX;
                if (front == min) y = armBaseLocationY;
                if (top == min) z = armBaseLocationZ;
            }
        }

        return collisionDetected;
    }
}