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
    public static final double SPEED_FULL = 0.6;
    public static final double SPEED_DRIVE = 0.40;
    public static final double SPEED_APPROACH = 0.25;

    public static final double SPEED_TURN = 0.5;
    public static final double ANGLE_90 = 90.0;
    public static final double TURN_LEFT = -ANGLE_90;
    public static final double TURN_RIGHT = ANGLE_90;
    public static final double WHITE_THRESHOLD = 0.6;  // background spans between 0.1 - 0.5 from dark to light

    public static final double SPEED_ARM = 0.25;
    public static final double TOUT_ARM = 5;

    public static final double TOUT_SHORT = 3;
    public static final double TOUT_MEDIUM = 5;
    public static final double TOUT_LONG = 10;

    /* Declare OpMode data members */
    MattSetupActuators robot = new MattSetupActuators();  // Use Pushbot's actuators
    MattSetupSensors sensors = new MattSetupSensors();    // Use Pushbot's sensors
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

    public void calibrateNoGyro() {
        telemetry.addData("Status", "calibrateNoGyro");
        telemetry.update();
        sensors.gyroSensor = null;
    }

    // Display the sensor levels while we are waiting to start
    public void waitForStartAndDisplayWhileWaiting() {
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Light Level  ", sensors.lightSensor.getLightDetected());
            telemetry.addData("Color:       ",
                    "RED " + sensors.colorSensor.red() +
                            "  GRN " + sensors.colorSensor.green() +
                            "  BLU " + sensors.colorSensor.blue()
            );
            telemetry.addData("Bumper Front ", sensors.touchSensorFront.isPressed());
            telemetry.addData("Bumper Arm   ", sensors.touchSensorArmPush.isPressed());
            telemetry.addData("Limit Switch ",
                    "IN " + sensors.touchSensorArmIn.isPressed() +
                            " OUT " + sensors.touchSensorArmOut.isPressed()
            );
            if (sensors.gyroSensor != null) {
                telemetry.addData("Gyro Z      ", sensors.gyroSensor.getIntegratedZValue());
            }
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
        if (speed < 0.0) {
            // speed for the encoderDrive(...) must be always positive!
            speed = -speed;
            degrees = -degrees;
        }
        encoderDrive(speed, degrees * MattSetupActuators.INCHES_PER_ANGLE_INPLACE, -degrees * MattSetupActuators.INCHES_PER_ANGLE_INPLACE, timeoutS);
    }

    public void encoderTurnAndDrag(double speed, double degrees, double timeoutS) {
        telemetry.addData("Status", "encoderTurnAndDrag");
        telemetry.update();
        if (speed < 0.0) {
            // speed for the encoderDrive(...) must be always positive!
            speed = -speed;
            degrees = -degrees;
        }
        if (degrees >= 0.0) {
            encoderDrive(speed, degrees * MattSetupActuators.INCHES_PER_ANGLE_DRAG, 0, timeoutS);
        } else {
            encoderDrive(speed, 0, -degrees * MattSetupActuators.INCHES_PER_ANGLE_DRAG, timeoutS);
        }
    }

    public void encoderDriveDistance(double speed, double distance, double timeoutS) {
        telemetry.addData("Status", "encoderDriveDistance");
        telemetry.update();
        if (speed < 0.0) {
            // speed for the encoderDrive(...) must be always positive!
            speed = -speed;
            distance = -distance;
        }
        encoderDrive(speed, distance, distance, timeoutS);
    }

    public void encoderDriveTime(double speed, double timeoutS) {
        telemetry.addData("Status", "encoderDriveTime for " + timeoutS);
        telemetry.update();
        runtime.reset(); // reset the timeout time and start motion.
        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);
        while (opModeIsActive() && (runtime.seconds() < timeoutS)) {
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
        while (opModeIsActive() && (sensors.touchSensorFront.isPressed() == false) && (runtime.seconds() < timeoutS)) {
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
        while (opModeIsActive() && (sensors.lightSensor.getLightDetected() < lightThreshold) && (runtime.seconds() < timeoutS)) {
            telemetry.addData("Status", "encoderDriveToWhiteLine");
            telemetry.addData("Light Level", sensors.lightSensor.getLightDetected());
            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }


    public void pushButton(double speed, double timeoutS) {
        telemetry.addData("Status", "pushButton - push");
        telemetry.update();
        if (speed < 0.0) speed = -speed; // we control the direction internally
        runtime.reset(); // reset the timeout time and start motion.
        robot.armMotor.setPower(-speed);
        while (opModeIsActive() &&
                (sensors.touchSensorArmPush.isPressed() == false) &&
                (sensors.touchSensorArmOut.isPressed() == false) &&
                (runtime.seconds() < timeoutS)) {
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        robot.armMotor.setPower(0);
        telemetry.addData("Status", "pushButton - sleep");
        sleep(500);
        telemetry.addData("Status", "pushButton - retract");
        telemetry.update();
        runtime.reset(); // reset the timeout time and start motion.
        robot.armMotor.setPower(speed);
        while (opModeIsActive() && (sensors.touchSensorArmIn.isPressed() == false) && (runtime.seconds() < timeoutS)) {
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
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * MattSetupActuators.COUNTS_PER_INCH);
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
                    (robot.leftMotor.isBusy() || robot.rightMotor.isBusy()) // && -> || --correction for the case of turn and drag
                    ) {

                // Display it for the driver.
                telemetry.addData("Status", "encoderDrive");
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
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
    private static final double HEADING_THRESHOLD = 1;       // As tight as we can make it with an integer gyro
    private static final double P_TURN_COEFF = 0.1;      // Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFF = 0.05;     // Larger is more responsive, but also less stable


    public void calibrateGyroOrFail(double timeoutS) {
        if (sensors.gyroSensor != null) {
            telemetry.addData("Status", "calibrateGyroOrFail in " + timeoutS);
            telemetry.update();
            try {
                runtime.reset(); // reset the timeout time and start motion.
                sensors.gyroSensor.calibrate();
                Boolean flash_color_led = true;
                while (!isStopRequested() && sensors.gyroSensor.isCalibrating()) {
                    if (runtime.seconds() > timeoutS) throw new TimeoutException();
                    sensors.colorSensor.enableLed(flash_color_led);
                    flash_color_led = !flash_color_led;
                    sleep(100);
                }
                telemetry.addData("Status", "calibrateGyroOrFail Done");
                telemetry.update();
            } catch (Exception e) {
                sensors.gyroSensor = null;
                telemetry.addData("Status", "calibrateGyroOrFail Failed");
                telemetry.update();
            }
        }
    }


    public void gyroResetHeading() {
        telemetry.addData("Status", "gyroResetHeading");
        telemetry.update();
        if (sensors.gyroSensor != null) sensors.gyroSensor.resetZAxisIntegrator();
    }


    public double gyroHeadingCorrection(double targetAngle) {
        double robotError = 0.0;
        if (sensors.gyroSensor != null) {
            try {
                // calculate error in -179 to +180 range  (
                robotError = targetAngle + sensors.gyroSensor.getIntegratedZValue();
                // Note: gyro is negated because left shoild be <0 and right should be >0
                // Note: this is correction and not error so we use (desired - actual)
                while (robotError > 180) robotError -= 360;
                while (robotError <= -180) robotError += 360;
            } catch (Exception e) {
                robotError = 0.0;
            }
        }
        return robotError;
    }

}