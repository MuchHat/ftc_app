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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// ************************** OP FOR TESTING THE SERVOS ******************************************//

@TeleOp(name = "Test Vu V9", group = "Test")
@Disabled
public class Test_Vu_V9 extends LinearOpMode {

    // ************************** VARIABLES ******************************************************//

    Vu vu = new Vu();
    private Team_Hardware_V9 robot = new Team_Hardware_V9();
    private ElapsedTime runtimeLoop = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    // ************************** OP LOOP ********************************************************//

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.colorBeacon.modeVuforiaFound = false;
        robot.colorBeacon.modeJewellSearch = true;
        robot.colorBeacon.displayStatus();

        telemetry.log().add("calibrating gyro, do not move");
        telemetry.update();
        sleep(111);

        runtimeLoop.reset();
        telemetry.log().clear();

        vu.init(hardwareMap);

        telemetry.clearAll(); // clear the telemetry view
        telemetry.addData(">", "press PLAY to start");
        telemetry.update(); // update the telemetry

        waitForStart(); // wait for the start button is pressed

        while (opModeIsActive()) {

            String targetSeenString = "seen: NO";
            if(vu.targetSeen())targetSeenString = "seen: YES";

            String targetVisibleString = "visible now: NO";
            if(vu.targetVisible())targetVisibleString = "visible now: YES";

            telemetry.addData("vuMark", "target %s", targetSeenString);
            telemetry.addData("vuMark", "target %s", targetVisibleString);
            telemetry.addData("vuMark", "target no", vu.getLastTargetSeenNo());
            telemetry.addData("vuMark", "target {%.2f %.2f}", vu.getX(), vu.getY());

            if (vu.targetSeen()) {
                robot.colorBeacon.modeVuforiaFound = vu.targetVisible();
            } else {
                robot.colorBeacon.modeVuforiaFound = true;
                robot.colorBeacon.displayStatus();
            }
            telemetry.update();
        }
    }
    // ************************** END OP *********************************************************//

    // ************************** END CLASS *******************************************************//
}

