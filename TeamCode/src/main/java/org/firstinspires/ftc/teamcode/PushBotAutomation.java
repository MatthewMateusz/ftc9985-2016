package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created on 11/25/2016.
 */

abstract public class PushBotAutomation extends LinearOpMode {

    /* Declare OpMode members. */
    MattSetupActuators robot   = new MattSetupActuators();  // Use Pushbot's actuators
    MattSetupSensors sensors = new MattSetupSensors();  // Use Pushbot's sensors
    // use the classes above that was created to define a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    public void setupHardware() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Status", "Init Auto One");    //
        telemetry.update();
        telemetry.addData("Status", "Init Mat Actuators");    //
        robot.init(hardwareMap);
        telemetry.addData("Status", "Init Mat Sensors");    //
        sensors.init(hardwareMap);
        telemetry.addData("Status", "Init Complete");    //
    }

    public void resetEncoders() {
        telemetry.addData("Status", "resetEncoders");  telemetry.update();
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void fullStop() {
        telemetry.addData("Status", "fullStop");  telemetry.update();
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        robot.armMotor.setPower(0);
    }

    public void turnInPlace(double speed, double degrees, double timeoutS) {
        telemetry.addData("Status", "turnInPlace");  telemetry.update();
        encoderDrive(speed, degrees*robot.INCHES_PER_ANGLE, -degrees*robot.INCHES_PER_ANGLE, timeoutS);
    }

    public void driveDistance(double speed, double inches, double timeoutS) {
        telemetry.addData("Status", "driveDistance");  telemetry.update();
        encoderDrive(speed, inches, inches, timeoutS);
    }

    public void driveToBumper(double speed, double maxdistance, double timeoutS) {
        telemetry.addData("Status", "driveToBumper");  telemetry.update();
        runtime.reset(); // reset the timeout time and start motion.
        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);
        while ( opModeIsActive() && (runtime.seconds() < timeoutS) && (sensors.touchSensorFront.isPressed()== false) )
        {
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void driveToWhiteLine(double speed, double lightThreshold, double maxdistance, double timeoutS) {
        telemetry.addData("Status", "driveToColor");  telemetry.update();
        runtime.reset(); // reset the timeout time and start motion.
        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);
        // Display the light level while we are looking for the line
        while ( opModeIsActive() && (runtime.seconds() < timeoutS) && (sensors.lightSensor.getLightDetected() < lightThreshold) )
        {
            telemetry.addData("Light Level",  sensors.lightSensor.getLightDetected()); telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void pushButton(double speed, double timeoutS) {
        telemetry.addData("Status", "pushButton");  telemetry.update();
        runtime.reset(); // reset the timeout time and start motion.
        robot.armMotor.setPower(-speed);
        while ( opModeIsActive() && (runtime.seconds() < timeoutS) && (sensors.touchSensorFront.isPressed()== false) )
        {
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        robot.armMotor.setPower(0);
        sleep(500);
        robot.armMotor.setPower(speed);
        sleep(100);
        robot.armMotor.setPower(0);
        idle();
    }



    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftInches * robot.COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * robot.COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}
