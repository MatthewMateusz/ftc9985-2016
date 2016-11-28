package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeoutException;

/**
 * Created on 11/25/2016.
 */

abstract public class PushBotAutomation extends LinearOpMode {


    /* Declare OpMode constants for users */
    public static final double     FULL_SPEED       = 1.0;
    public static final double     DRIVE_SPEED      = 0.75;
    public static final double     APPROACH_SPEED   = 0.5;

    public static final double     TURN_SPEED       = 0.5;
    public static final double     RIGHT_ANGLE      = 90.0;
    public static final double     TURN_LEFT        = -RIGHT_ANGLE;
    public static final double     TURN_RIGHT       = RIGHT_ANGLE;
    public static final double     WHITE_THRESHOLD  = 0.6;  // background spans between 0.1 - 0.5 from dark to light

    public static final double     ARM_SPEED        = 0.1;

    public static final double     SHORT_TIMEOUT    = 3;
    public static final double     MEDIUM_TIMEOUT   = 5;
    public static final double     LONG_TIMEOUT     = 10;

    /* Declare OpMode data members */
    MattSetupActuators robot   = new MattSetupActuators();  // Use Pushbot's actuators
    MattSetupSensors   sensors = new MattSetupSensors();  // Use Pushbot's sensors
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

    public void calibrateGyroOrFail(double timeoutS) {
        if (sensors.gyroSensor!=null) {
            telemetry.addData("Status", "Calibrating gyro for up to "+timeoutS);  telemetry.update();
            try {
                runtime.reset(); // reset the timeout time and start motion.
                sensors.gyroSensor.calibrate();
                Boolean flash_color_led = true;
                while (!isStopRequested() && sensors.gyroSensor.isCalibrating())  {
                    if (runtime.seconds() > timeoutS) throw new TimeoutException();
                    sensors.colorSensor.enableLed(flash_color_led); flash_color_led=!flash_color_led;
                    sleep(100);
                }
                telemetry.addData("Status", "Gyro calibration done");  telemetry.update();
            } catch (Exception e) {
                sensors.gyroSensor=null;
                telemetry.addData("Status", "Gyro calibration failed");  telemetry.update();
            }
        }
    }

    // Display the sensor levels while we are waiting to start
    public void waitForStartAndDisplayWhileWaiting() {
        while (!isStarted()) {
            telemetry.addData("Light Level ", sensors.lightSensor.getLightDetected());
            telemetry.addData("Red Level   ", sensors.colorSensor.red());
            telemetry.addData("Green Level ", sensors.colorSensor.green());
            telemetry.addData("Blue Level  ", sensors.colorSensor.blue());
            if (sensors.gyroSensor != null) telemetry.addData("Gyro Z      ", sensors.gyroSensor.getIntegratedZValue());
            telemetry.update();
            idle();
        }
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
        if ( speed<0.0 ) {
            // speed for the encoderDrive(...) must be always positive!
            speed=-speed;
            degrees=-degrees;
        }
        encoderDrive(speed, degrees* MattSetupActuators.INCHES_PER_ANGLE_INPLACE, -degrees* MattSetupActuators.INCHES_PER_ANGLE_INPLACE, timeoutS);
    }

    public void turnAndDrag(double speed, double degrees, double timeoutS) {
        telemetry.addData("Status", "turnAndDrag");  telemetry.update();
        if ( speed<0.0 ) {
            // speed for the encoderDrive(...) must be always positive!
            speed=-speed;
            degrees=-degrees;
        }
        if (degrees>=0.0) {
            encoderDrive(speed, degrees* MattSetupActuators.INCHES_PER_ANGLE_DRAG, 0, timeoutS);
        } else {
            encoderDrive(speed, 0, -degrees* MattSetupActuators.INCHES_PER_ANGLE_DRAG, timeoutS);
        }
    }

    public void driveDistance(double speed, double distance, double timeoutS) {
        telemetry.addData("Status", "driveDistance");  telemetry.update();
        if ( speed<0.0 ) {
            // speed for the encoderDrive(...) must be always positive!
            speed=-speed;
            distance=-distance;
        }
        encoderDrive(speed, distance, distance, timeoutS);
    }

    public void driveToBumper(double speed, double timeoutS) {
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

    public void driveToWhiteLine(double speed, double lightThreshold, double timeoutS) {
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
        if (speed<0.0) speed=-speed; // we control the direction internally
        runtime.reset(); // reset the timeout time and start motion.
        robot.armMotor.setPower(-speed);
        while ( opModeIsActive() && (runtime.seconds() < timeoutS) && (sensors.touchSensorFront.isPressed()== false) )
        {
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        robot.armMotor.setPower(0);
        sleep(500);
        robot.armMotor.setPower(speed);
        sleep(1500);
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
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(leftInches * MattSetupActuators.COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * MattSetupActuators.COUNTS_PER_INCH);
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
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    ( robot.leftMotor.isBusy() || robot.rightMotor.isBusy() ) // && -> || --correction for the case of turn and drag
                    ) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void startTicking() {
        // Reset the cycle clock for the next pass.
        period.reset();
    }

    /* local members. */
    private ElapsedTime period  = new ElapsedTime();

}
