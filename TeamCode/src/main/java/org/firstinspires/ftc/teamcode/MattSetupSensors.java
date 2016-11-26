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
    HardwareMap hardwareMap =  null;


    /* Constructor */
    public MattSetupSensors() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        //Define touchSensorFront
        touchSensorFront = (ModernRoboticsDigitalTouchSensor) hardwareMap.touchSensor.get("touchSensorFront");
        touchSensorArm   = (ModernRoboticsDigitalTouchSensor) hardwareMap.touchSensor.get("touchSensorArm");

        //Define lightSensor & enbale led
        lightSensor = hardwareMap.opticalDistanceSensor.get("lightSensor");
        lightSensor.enableLed(true);

    }


}
