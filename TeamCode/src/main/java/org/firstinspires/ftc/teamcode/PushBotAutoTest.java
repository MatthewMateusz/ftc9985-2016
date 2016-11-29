/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous Test", group="Auto")
public class PushBotAutoTest extends PushBotAutomation {


    /* Declare OpMode members. */
    // robot and sensors are declared in the super of this class - PushBotAutomation
    // and must be initialized by running setupHardware()

    // use the classes above that was created to define a Pushbot's hardware

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Status", "Init the automode");    //
        setupHardware();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");  telemetry.update();
        encoderReset();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d", robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition()); telemetry.update();

        // Try to calibrate the gyro if available and make sure it is calibrated before continuing or disable the gyro
        calibrateGyroOrFail(10);
        sensors.colorSensor.enableLed(false);

        // Display the sensor levels while we are waiting to start
        waitForStartAndDisplayWhileWaiting();
        telemetry.addData(">", "Robot Ready"); telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Test", "Started"); telemetry.update();

//      // Robot actuator constant calibration:
//      encoderDriveDistance(SPEED_DRIVE , 24.0,        TOUT_LONG);
//      encoderTurnInPlace(SPEED_TURN, ANGLE_90,        TOUT_MEDIUM);

//      // encoder Driving with sensors tests:
//      encoderDriveDistance(SPEED_DRIVE , 24.0,        TOUT_MEDIUM);
//      encoderDriveToBumper(SPEED_APPROACH,            TOUT_LONG);
//      encoderDriveDistance(SPEED_DRIVE , -2.0,        TOUT_LONG);
//      encoderDriveToWhiteLine(SPEED_APPROACH, WHITE_THRESHOLD, TOUT_LONG);

//      pushButton(SPEED_ARM, TOUT_MEDIUM);

        if (false) {
            // testing driving in cyrcles by encooder
            while (opModeIsActive()) {
                encoderTurnInPlace(SPEED_TURN, 40.0, TOUT_MEDIUM);
                sleep(500);
                encoderDriveDistance(SPEED_APPROACH, 6.0, TOUT_MEDIUM);
                sleep(500);
            }
        }

        if (true) {
            // testing driving in cyrcles by gyro
            while (opModeIsActive()) {
                encoderDriveDistance(SPEED_APPROACH, 12.0,      TOUT_MEDIUM);
                sleep(500);
                encoderTurnInPlace(SPEED_TURN, 90.0,            TOUT_MEDIUM);
                sleep(500);
                encoderDriveDistance(SPEED_APPROACH, 12.0,      TOUT_MEDIUM);
                sleep(500);
                encoderTurnInPlace(SPEED_TURN, -90,             TOUT_MEDIUM);
                sleep(500);
                encoderDriveDistance(SPEED_APPROACH, -12.0,     TOUT_MEDIUM);
                sleep(500);
                encoderTurnInPlace(SPEED_TURN, -90.0,           TOUT_MEDIUM);
                sleep(500);
                encoderDriveDistance(SPEED_APPROACH, 12.0,      TOUT_MEDIUM);
                sleep(500);
                encoderTurnInPlace(SPEED_TURN, 90.0,            TOUT_MEDIUM);
                sleep(500);
            }
        }


//      gyroDriveDistance(0.0, SPEED_DRIVE, 24.0,   TOUT_MEDIUM);
//      gyroDriveToBumper(0.0, SPEED_APPROACH,      TOUT_LONG);
//      gyroDriveDistance(0.0, SPEED_APPROACH, -2.0,TOUT_MEDIUM);
//      gyroTurnInPlace(90.0, SPEED_TURN,           TOUT_MEDIUM);
//      gyroDriveToWhiteLine(0.0, SPEED_APPROACH, WHITE_THRESHOLD, TOUT_LONG);

        if (false) {
            // testing driving in cyrcles by gyro
            while (opModeIsActive()) {
                gyroTurnInPlace(0.0, SPEED_TURN,                TOUT_MEDIUM);
                sleep(500);
                gyroDriveDistance(0.0, SPEED_APPROACH, 12.0,    TOUT_MEDIUM);
                sleep(500);
                gyroTurnInPlace(90.0, SPEED_TURN,               TOUT_MEDIUM);
                sleep(500);
                gyroDriveDistance(90.0, SPEED_APPROACH, 12.0,   TOUT_MEDIUM);
                sleep(500);
                gyroTurnInPlace(0.0, SPEED_TURN,                TOUT_MEDIUM);
                sleep(500);
                gyroDriveDistance(0.0, SPEED_APPROACH, -12.0,   TOUT_MEDIUM);
                sleep(500);
                gyroTurnInPlace(-90.0, SPEED_TURN,              TOUT_MEDIUM);
                sleep(500);
                gyroDriveDistance(-90, SPEED_APPROACH, 12.0,    TOUT_MEDIUM);
                sleep(500);
            }
        }




        telemetry.addData("Test", "Complete"); telemetry.update();
    }


}
