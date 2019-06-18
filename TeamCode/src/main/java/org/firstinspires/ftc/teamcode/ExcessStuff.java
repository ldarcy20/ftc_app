package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

public class ExcessStuff {
    static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    static public double elbowAngle(double currentPos) {
        //double finalPos = 90 - shoulderAngle((double) shoulderMotor.getCurrentPosition()) + 45 + ((currentPos + (2300 - 1576)) / (2300 * 4)) * 360;
        double finalPos = 82 + (double)currentPos/(2350*4)*360;
        return finalPos;
    }

    static public double shoulderAngle(double currentPos) {
        double finalPos = ((-currentPos + 4330) / (2350 * 4)) * 360;
        return finalPos;
    }
}
