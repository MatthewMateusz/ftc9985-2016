package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
            telemetry.update();
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
            telemetry.addData("Light Level",  sensors.lightSensor.getLightDetected());
            telemetry.update();
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
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);

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



    /* Declare OpMode members. */
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    private void gyroDrive ( double speed, double distance, double angle, double timeoutS) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * MattSetupActuators.COUNTS_PER_INCH);
            newLeftTarget = robot.leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            runtime.reset();
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&  (robot.leftMotor.isBusy() || robot.rightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftMotor.setPower(leftSpeed);
                robot.rightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    private void gyroTurn (  double speed, double angle, double timeoutS) {
        runtime.reset(); // reset the timeout time and start motion.
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF) && (runtime.seconds() < timeoutS)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    private void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        // Stop all motion;
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    private boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftMotor.setPower(leftSpeed);
        robot.rightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - sensors.gyroSensor.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


}

