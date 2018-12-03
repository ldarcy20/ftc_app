package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmCalculator {
    double elbowMotorSpeed;
    double shoulderMotorSpeed;
    double armOneLength;
    double armTwoLength;
    public ArmCalculator(double lengthOne, double lengthTwo){
        armOneLength = lengthOne;
        armTwoLength = lengthTwo;
    }
    public void calculateSpeed(double joystickX, double joystickY, DcMotor shoulderMotor, DcMotor elbowMotor){
        double verticalAngleVelocity = joystickY;
        double scale = (-1*(armOneLength*Math.cos(shoulderMotor.getCurrentPosition()) + armTwoLength*Math.sin(elbowMotor.getCurrentPosition() + shoulderMotor.getCurrentPosition() - 90))/(armTwoLength*Math.sin( elbowMotor.getCurrentPosition() + shoulderMotor.getCurrentPosition() - 90)));
        double moveAcross = joystickX;
        double moveUp = moveAcross * (1/scale) + verticalAngleVelocity;
        //double ratio = moveUp/moveAcross;
        if(Math.abs(moveUp) > Math.abs(moveAcross)){
            moveAcross = moveAcross*(1/Math.abs(moveUp));
            moveUp = moveUp*(1/Math.abs(moveUp));
        }
        else{
            moveUp = moveUp*(1/Math.abs(moveAcross));
            moveAcross = moveAcross*(1/Math.abs(moveAcross));
        }

      /*  if(ratio > 1 && moveup > 0){
            shoulderMotorPower = 1;
            elbowMotorPower = 1/ ratio;
        }
        else if (ratio < -1 && moveup < 0){
            shoulderMotorPower = -1;
            elbowMotorPower = -1/ ratio;
        }
        else if (ratio > 1 && moveup < 0){
            shoulderMotorPower = -1;
            elbowMotorPower = -1/ ratio;
        }
        else if (ratio < -1 && moveup > 0){
            shoulderMotorPower = 1;
            elbowMotorPower = 1/ ratio;
        }
        else if (ratio > 0 && moveup > 0){
            shoulderMotorPower = ratio;
            elbowMotorPower = 1;
        }
        else if (ratio > 0 && moveup < 0){
            shoulderMotorPower = -ratio;
            elbowMotorPower = -1;
        }
        else if (ratio < 0 && moveup > 0){
            shoulderMotorPower = - ratio;
            elbowMotorPower = -1;
        }
        else {
            shoulderMotorPower = ratio;
            elbowMotorPower = 1;
        }*/
        shoulderMotorSpeed = moveUp;
        elbowMotorSpeed = moveAcross;
    }

    public double getElbowMotorSpeed() {
        return elbowMotorSpeed;
    }

    public double getShoulderMotorSpeed() {
        return shoulderMotorSpeed;
    }


    
}
