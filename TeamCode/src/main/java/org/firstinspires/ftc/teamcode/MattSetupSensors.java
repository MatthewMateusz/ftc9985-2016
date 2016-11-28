package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created on 11/25/2016.
 */

public class MattSetupSensors {

    /* Public sensor members. */
    /* Touch Sensor */
    public ModernRoboticsDigitalTouchSensor touchSensorFront = null;
    public ModernRoboticsDigitalTouchSensor touchSensorArm = null;

    //ODS *Addon
    public OpticalDistanceSensor lightSensor = null;

    //MRColor Sensor *Addon
    public ColorSensor colorSensor = null;

    public ModernRoboticsI2cGyro gyroSensor = null;

    /* local members. */
    HardwareMap hwMap =  null;


    /* Constructor */
    public MattSetupSensors() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //Define touchSensorFront
        touchSensorFront = (ModernRoboticsDigitalTouchSensor) hwMap.touchSensor.get("touchSensorFront");
        touchSensorArm   = (ModernRoboticsDigitalTouchSensor) hwMap.touchSensor.get("touchSensorArm");

        //Define lightSensor & enbale led
        lightSensor = hwMap.opticalDistanceSensor.get("lightSensor");
        lightSensor.enableLed(true); // brightness of reflected lioght is measured

        // Define ColorSensor & enbale led
        colorSensor = hwMap.colorSensor.get("sensor_color");
        colorSensor.enableLed(false); // detect the shining light, not reflected

        // get a reference to a Modern Robotics GyroSensor object if available
        try {
            gyroSensor = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");
            gyroSensor.resetZAxisIntegrator();
        } catch (Exception e) {
            gyroSensor = null;
        }
    }


}
