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
    public static final double  SPEED_FULL      = 1.0;
    public static final double  SPEED_DRIVE     = 0.75;
    public static final double  SPEED_APPROACH  = 0.5;

    public static final double  SPEED_TURN      = 0.5;
    public static final double  ANGLE_90        = 90.0;
    public static final double  TURN_LEFT       = -ANGLE_90;
    public static final double  TURN_RIGHT      = ANGLE_90;
    public static final double  WHITE_THRESHOLD = 0.6;  // background spans between 0.1 - 0.5 from dark to light

    public static final double  SPEED_ARM       = 0.1;
    public static final double  TOUT_ARM        = 5;

    public static final double  TOUT_SHORT      = 3;
    public static final double  TOUT_MEDIUM     = 5;
    public static final double  TOUT_LONG       = 10;

    /* Declare OpMode data members */
    MattSetupActuators robot   = new MattSetupActuators();  // Use Pushbot's actuators
    MattSetupSensors   sensors = new MattSetupSensors();    // Use Pushbot's sensors
    /* Timeout variable */
    private ElapsedTime runtime = new ElapsedTime();

    public void setupHardware() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Status", "Init Automation");
        telemetry.addData("Status", "Init Mat Actuators");
        telemetry.update();
        robot.init(hardwareMap);
        telemetry.addData("Status", "Init Automation");
        telemetry.addData("Status", "Init Mat Sensors");
        sensors.init(hardwareMap);
        telemetry.addData("Status", "Init Automation Done");
        telemetry.update();
    }

    public void calibrateGyroOrFail(double timeoutS) {
        if (sensors.gyroSensor!=null) {
            telemetry.addData("Status", "calibrateGyroOrFail in "+timeoutS);
            telemetry.update();
            try {
                runtime.reset(); // reset the timeout time and start motion.
                sensors.gyroSensor.calibrate();
                Boolean flash_color_led = true;
                while (!isStopRequested() && sensors.gyroSensor.isCalibrating())  {
                    if (runtime.seconds() > timeoutS) throw new TimeoutException();
                    sensors.colorSensor.enableLed(flash_color_led); flash_color_led=!flash_color_led;
                    sleep(100);
                }
                telemetry.addData("Status", "calibrateGyroOrFail Done");
                telemetry.update();
            } catch (Exception e) {
                sensors.gyroSensor=null;
                telemetry.addData("Status", "calibrateGyroOrFail Failed");
                telemetry.update();
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

    public void fullStop() {
        telemetry.addData("Status", "fullStop");
        telemetry.update();
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        robot.armMotor.setPower(0);
    }


    public void encoderReset() {
        telemetry.addData("Status", "encoderReset");
        telemetry.update();
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderTurnInPlace(double speed, double degrees, double timeoutS) {
        telemetry.addData("Status", "encoderTurnInPlace");
        telemetry.update();
        if ( speed<0.0 ) {
            // speed for the encoderDrive(...) must be always positive!
            speed=-speed;
            degrees=-degrees;
        }
        encoderDrive(speed, degrees* MattSetupActuators.INCHES_PER_ANGLE_INPLACE, -degrees* MattSetupActuators.INCHES_PER_ANGLE_INPLACE, timeoutS);
    }

    public void encoderTurnAndDrag(double speed, double degrees, double timeoutS) {
        telemetry.addData("Status", "encoderTurnAndDrag");
        telemetry.update();
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

    public void encoderDriveDistance(double speed, double distance, double timeoutS) {
        telemetry.addData("Status", "encoderDriveDistance");
        telemetry.update();
        if ( speed<0.0 ) {
            // speed for the encoderDrive(...) must be always positive!
            speed=-speed;
            distance=-distance;
        }
        encoderDrive(speed, distance, distance, timeoutS);
    }

    public void encoderDriveTime(double speed, double timeoutS) {
        telemetry.addData("Status", "encoderDriveTime for " + timeoutS);
        telemetry.update();
        runtime.reset(); // reset the timeout time and start motion.
        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);
        while ( opModeIsActive() && (runtime.seconds() < timeoutS) )
        {
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void encoderDriveToBumper(double speed, double timeoutS) {
        telemetry.addData("Status", "encoderDriveToBumper");
        telemetry.update();
        runtime.reset(); // reset the timeout time and start motion.
        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);
        while ( opModeIsActive() && (sensors.touchSensorFront.isPressed()== false) && (runtime.seconds() < timeoutS) )
        {
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void encoderDriveToWhiteLine(double speed, double lightThreshold, double timeoutS) {
        telemetry.addData("Status", "encoderDriveToWhiteLine");
        telemetry.update();
        runtime.reset(); // reset the timeout time and start motion.
        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);
        // Display the light level while we are looking for the line
        while ( opModeIsActive() && (sensors.lightSensor.getLightDetected() < lightThreshold) && (runtime.seconds() < timeoutS) )
        {
            telemetry.addData("Status", "encoderDriveToWhiteLine");
            telemetry.addData("Light Level",  sensors.lightSensor.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }


    public void pushButton(double speed, double timeoutS) {
        telemetry.addData("Status", "pushButton");
        telemetry.update();
        if (speed<0.0) speed=-speed; // we control the direction internally
        runtime.reset(); // reset the timeout time and start motion.
        robot.armMotor.setPower(-speed);
        while ( opModeIsActive() && (runtime.seconds() < timeoutS) && (sensors.touchSensorArmPush.isPressed()== false) )
        {
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        robot.armMotor.setPower(0);
        sleep(500);
        robot.armMotor.setPower(speed);
        while ( opModeIsActive() && (runtime.seconds() < timeoutS) && (sensors.touchSensorArmIn.isPressed()== false) )
        {
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
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
                telemetry.addData("Status", "encoderDrive");
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition());
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



    /* Declare OpMode members. */
    private static final double     HEADING_THRESHOLD       = 1 ;       // As tight as we can make it with an integer gyro
    private static final double     P_TURN_COEFF            = 0.1;      // Larger is more responsive, but also less stable
    private static final double     P_DRIVE_COEFF           = 0.05;     // Larger is more responsive, but also less stable


    public void gyroResetHeading() {
        telemetry.addData("Status", "gyroResetHeading");
        telemetry.update();
        if (sensors.gyroSensor!=null) sensors.gyroSensor.resetZAxisIntegrator();
    }


    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (heading)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param heading      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative heading is required, add/subtract from current heading.
     */
    public void gyroTurnInPlace(double heading, double speed, double timeoutS) {
        if (sensors.gyroSensor==null) {
            // scale back to encoder drive
            encoderTurnInPlace(speed, (heading-oldheading), timeoutS);
            oldheading = heading;
        } else {
            runtime.reset(); // reset the timeout time and start motion.
            // keep looping while we are still active, and not on heading.
            while (opModeIsActive() && !gyroOneStepTurn(heading, speed, P_TURN_COEFF) && (runtime.seconds() < timeoutS)) {
                // Update telemetry & Allow time for other processes to run.
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
        }
    }
    /* variable to */
    private double oldheading = 0.0;

    /**
     *  Method to drive on a fixed compass bearing (heading), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param heading      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative heading is required, add/subtract from current heading.
     */
    public void gyroDriveDistance(double heading, double speed, double distance, double timeoutS) {
        if (sensors.gyroSensor==null) {
            // scale back to encoder drive
            encoderDriveDistance(speed, distance, timeoutS);
            return;
        } // else the reminder of this function

        if (speed<0.0) {
            speed = -speed;
            distance = -distance;
        }

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
                error = gyroHeadingError(heading);
                steer = gyroComputeSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) steer *= -1.0;

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
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param heading      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative heading is required, add/subtract from current heading.
     * @param timeoutS   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroDriveTime(double heading, double speed, double timeoutS) {
        if (sensors.gyroSensor==null) {
            // scale back to encoder drive
            encoderDriveTime(speed, timeoutS);
        } else {
            // reset the timeout time and start motion.
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
                // Update telemetry & Allow time for other processes to run.
                gyroOneStepDrive(heading, speed, P_DRIVE_COEFF);
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }
    }

    public void gyroDriveToBumper(double heading, double speed, double timeoutS) {
        if (sensors.gyroSensor==null) {
            // scale back to encoder drive
            encoderDriveToBumper(speed, timeoutS);
        } else {
            // reset the timeout time and start motion.
            runtime.reset();
            while (opModeIsActive() && (sensors.touchSensorFront.isPressed()== false) && (runtime.seconds() < timeoutS)) {
                // Update telemetry & Allow time for other processes to run.
                gyroOneStepDrive(heading, speed, P_DRIVE_COEFF);
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }
    }

    public void gyroDriveToWhiteLine(double heading, double speed, double lightThreshold, double timeoutS) {
        if (sensors.gyroSensor==null) {
            // scale back to encoder drive
            encoderDriveToWhiteLine(speed, lightThreshold, timeoutS);
        } else {
            // reset the timeout time and start motion.
            runtime.reset();
            while (opModeIsActive() && (sensors.lightSensor.getLightDetected() < lightThreshold) && (runtime.seconds() < timeoutS)) {
                // Update telemetry & Allow time for other processes to run.
                gyroOneStepDrive(heading, speed, P_DRIVE_COEFF);
                telemetry.addData("Light Level",  sensors.lightSensor.getLightDetected());
                telemetry.update();
                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            }
            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }


    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param heading     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative heading is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    private boolean gyroOneStepTurn(double heading, double speed, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = gyroHeadingError(heading);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = gyroComputeSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftMotor.setPower(leftSpeed);
        robot.rightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", heading);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }


    /**
     * Perform one cycle of closed loop heading control for driving (not turning)
     *
     * @param speed     Desired speed of turn.
     * @param heading   Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative heading is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    private void gyroOneStepDrive(double heading, double speed, double PCoeff) {
        double  error ;
        double  steer ;
        double  max;
        double  leftSpeed;
        double  rightSpeed;

        // adjust relative speed based on heading error.
        error = gyroHeadingError(heading);
        steer = gyroComputeSteer(error, P_DRIVE_COEFF);

        // if driving in reverse, the motor correction also needs to be reversed
        if (speed < 0) steer *= -1.0;

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

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", heading);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
    }

    /**
     * gyroHeadingError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    private double gyroHeadingError(double targetAngle) {

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
    private double gyroComputeSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


}

