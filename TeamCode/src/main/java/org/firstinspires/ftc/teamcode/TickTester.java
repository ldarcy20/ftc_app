package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp(name = "Tick Tester")
public class TickTester extends OpMode {
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx middleMotor;
    DcMotorEx middleMotor2;
    @Override
    public void init() {
        /** Wait for the game to begin */
        leftMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"leftMotor");
        rightMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "rightMotor");
        middleMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "middleMotor");
        middleMotor2 = (DcMotorEx)hardwareMap.get(DcMotor.class,"middleMotor2");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        middleMotor.setDirection(DcMotor.Direction.FORWARD);
        middleMotor2.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        telemetry.addData("left Motor", leftMotor.getCurrentPosition());
        telemetry.addData("right Motor", rightMotor.getCurrentPosition());
        telemetry.addData("middle Motor", middleMotor.getCurrentPosition());
        telemetry.addData("middle Motor 2", middleMotor2.getCurrentPosition());
        telemetry.update();
    }
}
