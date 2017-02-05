package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Decide whether the detected color is more RED or more BLUE
 * in the challenging lighting conditions of the competition
 */

public class MattColorDetector {
    public static boolean confirmBlue(ColorSensor color) {
        return( color.blue() > color.red() );
    }


    public static boolean confirmRed(ColorSensor color) {
        return( color.blue() <= color.red() );
    }

}
