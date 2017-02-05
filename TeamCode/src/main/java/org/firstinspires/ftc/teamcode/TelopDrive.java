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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.util.Range;


/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleoperation", group="Teleop")
public class TelopDrive extends OpMode{

    /* Declare OpMode members. */
    private MattSetupActuators robot            = new MattSetupActuators(); // Use Pushbot's actuators
    private MattSetupSensors sensors            = new MattSetupSensors();   // Use Pushbot's sensors
    // use the class above that was created to define a Pushbot's hardware
//  private double              clawOffset      = 0.0 ;                     // Servo mid position

    //  private staticfinal double  CLAW_SPEED      = 0.02 ;                    // sets rate to move servo
    private static final double ARM_UP_POWER    =  0.8 ;                   //
    private static final double ARM_DOWN_POWER  = -0.8 ;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        sensors.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double left   = -gamepad1.left_stick_y;
        double right  = -gamepad1.right_stick_y;
        double middle = -gamepad2.left_stick_y;

        // precision mode drive
        if (gamepad1.left_bumper || gamepad1.right_bumper)
        {
            left = 0.25 * left;
            right = 0.25 * right;
        }

        // crash avoidane system, override it by presssong B
        if ( (left>0.0) && (right>0.0) && sensors.touchSensorFront.isPressed() && !gamepad1.b ) {
            left = 0.0;
            right = 0.0;
        }

        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);


        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (Math.abs(middle)<0.1 ) {
            if (gamepad1.y) {
                middle = ARM_UP_POWER;
            } else if (gamepad1.a) {
                middle = ARM_DOWN_POWER;
            } else {
                robot.armMotor.setPower(0.0);
            }
        }

        // Limit arm movement at end postions or when pushed enough
        if (  ( (middle<0) &&
                ( sensors.touchSensorArmPush.isPressed() ||sensors.touchSensorArmOut.isPressed()))
                ||( (middle>0) && sensors.touchSensorArmIn.isPressed() ) ) {
            middle=0;
        }

        robot.armMotor.setPower(middle);


//        // Use gamepad left & right Bumpers to open and close the claw
//        if (gamepad1.right_bumper) clawOffset += CLAW_SPEED;
//        else if (gamepad1.left_bumper) clawOffset -= CLAW_SPEED;
//        clawOffset = Range.clip(clawOffset, -0.5, 0.5);


        // Send telemetry message to signify robot running;
//        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("left",       "%.2f",     left);
        telemetry.addData("right",      "%.2f",     right);
        telemetry.addData("arm",        "%.2f",     middle);
        telemetry.addData("left trig",  "%.2f",     gamepad1.left_trigger);
        telemetry.addData("right trig",  "%.2f",    gamepad1.right_trigger);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Say", "Bye Driver");    //
    }

}
