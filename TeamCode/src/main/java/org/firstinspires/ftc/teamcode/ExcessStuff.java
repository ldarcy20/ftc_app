package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

public class ExcessStuff {
    static boolean firstTime = true;
    static double startPos;
    static boolean finished = true;

    static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    static public double elbowAngle(double currentPos) {
        //double finalPos = 90 - shoulderAngle((double) shoulderMotor.getCurrentPosition()) + 45 + ((currentPos + (2300 - 1576)) / (2300 * 4)) * 360;
        double finalPos = 80 + (double)currentPos/(2350*4)*360;
        return finalPos;
    }

    static public double shoulderAngle(double currentPos) {
        double finalPos =  152 - (currentPos/((4318-1758)/90));
        return finalPos;
    }
    static public double scaleSpeed (double maxSpeed, double minSpeed, double targetPos, double currentPos){

        if (firstTime){
            startPos = currentPos;
            firstTime = false;
            finished = false;
        }
        double speedDif = maxSpeed - minSpeed;
        double dis = targetPos - startPos;
        double midPt = startPos + dis/2;
        if(currentPos >= startPos && currentPos <= midPt){
            return minSpeed + ((currentPos- startPos)/(dis/2))*(speedDif);
        }
        else if (currentPos <= targetPos && currentPos > midPt){
            return minSpeed + ((targetPos - currentPos)/(dis/2))*(speedDif);
        }
        if (currentPos >= targetPos){
            firstTime = true;
            finished = true;
        }
        return 0;
    }
    static public boolean closeEnough(double currentPos, double targetPos, double tolerance){
        if(Math.abs(targetPos - currentPos) > tolerance){
            return false;
        }
        return true;
    }

}
