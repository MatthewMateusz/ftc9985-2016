package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by olekmali on 1/4/2017.
 */

public class MattColorDetector {
    public static boolean confirmBlue(ColorSensor color) {
        return( color.blue() > color.red() );
    }


    public static boolean confirmRed(ColorSensor color) {
        return( color.blue() <= color.red() );
    }

}
