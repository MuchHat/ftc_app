/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

// ************************** OP FOR TESTING THE SERVOS ******************************************//

@TeleOp(name = "Test Vuforia V1", group = "Team")
//@Disabled
public class Team_TestVuforia_V1 extends LinearOpMode {

    // ************************** VARIABLES ******************************************************//

    private Team_Hardware_V3 robot = new Team_Hardware_V3();

    private ElapsedTime runtimeLoop = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();

    VuforiaLocalizer vuforia;
    // ************************** OP LOOP ********************************************************//

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.colorBeacon.yellow();

        telemetry.log().add("calibrating gyro, do not move");
        telemetry.update();
        sleep(111);
        robot.modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        runtimeLoop.reset();
        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating gyro", "%s",
                    Math.round(runtimeLoop.seconds()) % 2 == 0 ? "..  " : "   ..");
            telemetry.update();
            sleep(66);
        }
        runtimeLoop.reset();
        telemetry.log().clear();

        //**************************** VUFORIA INIT ***********************************************//
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Ac6cr63/////AAAAGUsQTEyyG0kggwF13U8WoMlPgXZiUoKR9pf2nlfhVVfvDXFsTn0wufoywxzibq+y5BGBa2cChKWAcUkKaD9/ak5lwCm9Wp3Osk9omsMR0YYoxt4TuPktrflK4HuTH8cOAQA8YDuOs/SO/cgOmWbQZtRXN/lFkUwGZA9eiV5D8730BG2SBLPR4A9rcFs0Fp/yPgcm4Zsh5Kv2Ct8XjJXmXk5mAjERZ5B6hKQzf/4wd9tSQ6BeQLvsgd5nI0Pj+K1NHI4EyHdFyxCPu91AMcCsXCLjkABfYt11Zhxu1uYaFF/AcN3eBHRwprVpDEBBXOMnD4BRCj0xxYYPWWO6g4gcjqBPgBos5nCDk43KipEeX22z";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = true; //TODO
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        //**************************** VUFORIA END INIT ********************************************//

        telemetry.clearAll(); // clear the telemetry view
        telemetry.addData(">", "press PLAY to start");
        telemetry.update(); // update the telemetry

        waitForStart(); // wait for the start button is pressed
        relicTrackables.activate();

        while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("vuMark", "%s visible", vuMark);
                robot.colorBeacon.green();

                /* For fun, we also exhibit the navigational pose */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;

                    // should be the direct distance
                    double mag = trans.magnitude();

                    telemetry.addData("pose t", "{%.2f, %.2f, %.2f}", tX, tY, tZ);
                    telemetry.addData("pose r", "{%.2f, %.2f, %.2f}", rX, rY, rZ);
                    telemetry.addData("mag", "{%.2f}", mag);
                }
            } else {
                telemetry.addData("vuMark", "not visible");
                robot.colorBeacon.yellow();
            }
            telemetry.update();
        }
    }
    // ************************** END OP *********************************************************//

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    // ************************** END CLASS *******************************************************//
}

