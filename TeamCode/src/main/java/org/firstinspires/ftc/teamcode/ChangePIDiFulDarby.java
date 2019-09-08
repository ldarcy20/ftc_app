package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Darby is petty")
public class ChangePIDiFulDarby extends OpMode {
    PIDFCoefficients pidStuff;
    double p;
    double i;
    double d;
    double f;
    int mode = 0;
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx middleMotor;
    DcMotorEx middleMotor2;
    boolean right_trigger_isPressed;
    boolean right_bumper_isPressed;
    boolean left_bumper_isPressed;

    @Override
    public void init() {
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotor");
        middleMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor");
        middleMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor2");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor2.setDirection(DcMotor.Direction.REVERSE);
        pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff.p = 10;
        pidStuff.i = 2;
        pidStuff.d = 0;
        pidStuff.f = 14;
        p = 10;
        i = 2;
        d = 0;
        f = 14;
        pidStuff.algorithm = MotorControlAlgorithm.PIDF;
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        right_trigger_isPressed = false;
        right_bumper_isPressed = false;
        left_bumper_isPressed = false;

    }

    @Override
    public void loop() {
        if (gamepad1. right_trigger > 0 && !right_trigger_isPressed){
            mode += 1;
            mode = mode%4;
            right_trigger_isPressed = true;
        }
        if (gamepad1.right_trigger == 0){
            right_trigger_isPressed = false;
        }
        if (mode == 0){
            changeValue(p, 1);
        }
        else if (mode == 1){
            changeValue(i, .25);
        }
        else if (mode == 2){
            changeValue(d, .25);
        }
        else if (mode == 3){
            changeValue(f, 1);
        }
        pidStuff.p = p;
        pidStuff.i = i;
        pidStuff.d = d;
        pidStuff.f = f;
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        telemetry.addData( "P:", pidStuff.p);
        telemetry.addData( "I:", pidStuff.i);
        telemetry.addData( "D:", pidStuff.d);
        telemetry.addData( "F:", pidStuff.f);


    }
    public void changeValue(double daniel, double incr){
        if (gamepad1.left_bumper && !left_bumper_isPressed){
            daniel -= incr;
            left_bumper_isPressed = true;
        }
        if (gamepad1.right_bumper && !right_bumper_isPressed){
            daniel += incr;
            right_bumper_isPressed = true;
        }
        if (!gamepad1.right_bumper){
            right_bumper_isPressed = false;
        }
        if (!gamepad1.left_bumper){
            left_bumper_isPressed = false;
        }
    }
}
