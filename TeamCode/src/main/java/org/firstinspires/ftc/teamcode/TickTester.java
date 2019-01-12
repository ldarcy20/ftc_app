package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name = "Tick Tester")
public class TickTester extends OpMode {
    static final double     COUNTS_PER_MOTOR_REV    = 1150 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4; ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_SIDE = 176;
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx middleMotor;
    DcMotorEx middleMotor2;
    DcMotorEx hangArm;
    Servo hangArmLock;
    String angleDouble = "hi";
    Orientation angles;
    BNO055IMU imu;
    PIDFCoefficients pidStuff;
    boolean state = false;
    int counter = 150;
    int leftEncoders = 0;
    int rightEncoders = 0;
    int middleEncoders = 0;

    int leftEncodersFinal = 0;
    int rightEncodersFinal = 0;
    int middleEncodersFinal = 0;

    int leftChange = 0;
    int rightChange = 0;
    int middleChange = 0;

    double leftChangeInches = 0;
    double rightChangeInches = 0;
    double middleChangeInches = 0;

    @Override
    public void init() {
        /** Wait for the game to begin */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        leftMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"leftMotor");
        rightMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "rightMotor");
        middleMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "middleMotor");
        middleMotor2 = (DcMotorEx)hardwareMap.get(DcMotor.class,"middleMotor2");
        hangArm = (DcMotorEx)hardwareMap.get(DcMotor.class, "Hang Arm");
        hangArmLock = hardwareMap.servo.get("Hang Arm Lock");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        middleMotor.setDirection(DcMotor.Direction.FORWARD);
        middleMotor2.setDirection(DcMotor.Direction.FORWARD);
        pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff.p = 0;
        pidStuff.i = 0;
        pidStuff.d = 0;
        pidStuff.f = 13;
        pidStuff.algorithm = MotorControlAlgorithm.PIDF;
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidStuff);
        middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        hangArm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);

    }

    @Override
    public void loop() {
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        leftMotor.setPower(gamepad1.left_stick_y);
        rightMotor.setPower(gamepad1.left_stick_y);
        middleMotor.setPower(gamepad1.left_stick_x);
        middleMotor2.setPower(gamepad1.left_stick_x);


        /*if(gamepad1.a) {
            double leftPower = .3 + .5*Math.cos((Double.parseDouble(angleDouble)));
            double rightPower = -.3 + .5*Math.cos((Double.parseDouble(angleDouble)));
            double middlePower = .5*Math.sin(Double.parseDouble(angleDouble));
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);
            middleMotor.setPower(middlePower);
        }
        telemetry.addData("left Motor", leftMotor.getCurrentPosition());
        telemetry.addData("right Motor", rightMotor.getCurrentPosition());
        telemetry.addData("middle Motor", middleMotor.getCurrentPosition());
        telemetry.addData("middle Motor 2", middleMotor2.getCurrentPosition());
        telemetry.addData("angle", angleDouble);
        double realAngle = convertAngle(angleDouble);
        telemetry.addData("Real Angle", realAngle);
        telemetry.update();*/
        if(gamepad1.a && state == false && counter > 150) {
            state = true;
            counter = 0;
            leftEncoders = leftMotor.getCurrentPosition();
            rightEncoders = rightMotor.getCurrentPosition();
            middleEncoders = middleMotor.getCurrentPosition();
        }
        else if(gamepad1.a && state == true && counter > 150) {
            state = false;
            counter = 0;
            leftEncodersFinal = leftMotor.getCurrentPosition();
            rightEncodersFinal = rightMotor.getCurrentPosition();
            middleEncodersFinal = middleMotor.getCurrentPosition();

            leftChange = leftEncodersFinal - leftEncoders;
            rightChange = rightEncodersFinal - rightEncoders;
            middleChange = middleEncodersFinal - middleEncoders;

            leftChangeInches = leftChange/COUNTS_PER_INCH;
            rightChangeInches = rightChange/COUNTS_PER_INCH;
            middleChangeInches = middleChange/COUNTS_PER_INCH;
            telemetry.addData("Left Change", leftChange);
            telemetry.addData("Right Change", rightChange);
            telemetry.addData("Middle Change", middleChange);
            telemetry.addData("Left Inches", leftChangeInches);
            telemetry.addData("Right Inches", rightChangeInches);
            telemetry.addData("Middle Inches", middleChangeInches);
            telemetry.update();
        }
        telemetry.addData("Middle", middleMotor.getCurrentPosition());
        telemetry.addData("Angle", angleDouble);
        telemetry.addData("Hang Arm", hangArm.getCurrentPosition());
        telemetry.addData("Hang Arm Lock", hangArmLock.getPosition());

        //telemetry.addData("Counts", leftMotor.get);
        telemetry.update();
        //telemetry.addData("Counter",counter);
        //telemetry.update();
        counter++;
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public double convertAngle(String angle) {
        double angleUsed = Double.parseDouble(angle);
        if(angleUsed < 0) {
            angleUsed = 180 + (180-Math.abs(angleUsed));
        }
        else {
            angleUsed = angleUsed;
        }
        return angleUsed;
    }
}
