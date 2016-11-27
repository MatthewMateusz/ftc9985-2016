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
    ModernRoboticsDigitalTouchSensor touchSensorFront = null;
    ModernRoboticsDigitalTouchSensor touchSensorArm = null;

    //ODS *Addon
    OpticalDistanceSensor lightSensor = null;

    //MRColor Sensor *Addon
    ColorSensor colorSensor = null;

    ModernRoboticsI2cGyro gyroSensor = null;

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
        lightSensor = hwMap.opticalDistanceSensor.get("sensor_light");
        lightSensor.enableLed(true);

        // Define ColorSensor & enbale led
        colorSensor = hwMap.colorSensor.get("colorSensor");
        colorSensor.enableLed(true);

        // get a reference to a Modern Robotics GyroSensor object if available
        try {
            gyroSensor = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");
            gyroSensor.resetZAxisIntegrator();
        } catch (Exception e) {
            gyroSensor = null;
        }
    }


}
