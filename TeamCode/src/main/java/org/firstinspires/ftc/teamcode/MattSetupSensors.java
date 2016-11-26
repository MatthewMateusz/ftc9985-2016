package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
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
    OpticalDistanceSensor lightSensor;

    //MRColor Sensor *Addon
    ColorSensor colorSensor;

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
        lightSensor.enableLed(true);

        // Define ColorSensor & enbale led
        colorSensor = hwMap.colorSensor.get("colorSensor");
        colorSensor.enableLed(true);
    }


}
