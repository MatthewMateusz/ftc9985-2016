package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:       Left  drive motor:        "left_drive"
 * Motor channel:       Right drive motor:        "right_drive"
 * Motor channel:       Manipulator drive motor:  "left_arm"
 * N/A Servo channel:  Servo to open left claw:  "left_hand"
 * N/A Servo channel:  Servo to open right claw: "right_hand"
 */
public class MattSetupActuators
{
    /* Public actuator members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor  armMotor    = null;


    public static final double COUNTS_PER_MOTOR_REV     = 1440;    // eg: TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION     = 2.0;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES    = 4.0 * (24.5/24.0);  // wheel circumference with real-life travel distance adjustment
    public static final double WHEEL_SEPARATION_INCHES  = 14.0;
    public static final double COUNTS_PER_INCH          = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159269);
    public static final double INCHES_PER_ANGLE_INPLACE = (11.5 / 90.0); // calibration affected by the distance between the driving wheels
    public static final double INCHES_PER_ANGLE_DRAG    = ( (WHEEL_SEPARATION_INCHES * 3.14159269 / 2.0) / 90.0); // calibration affected by the distance between the driving wheels

    /* local members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public MattSetupActuators(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");
        armMotor    = hwMap.dcMotor.get("left_arm");
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        // -- none at this time
    }

}
