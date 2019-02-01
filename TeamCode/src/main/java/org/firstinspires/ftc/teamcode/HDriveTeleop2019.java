package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;
// * Created by definitly not HIRSH as he would mess it up and it would explode on 8/18/2016

@TeleOp(name = "Rover Ruckus TeleOp")
public class HDriveTeleop2019 extends OpMode {
    //HDrive2 calculator
    HDriveFCCalc calculator;
    ArmCalculator armCalculator;

    //Ints, Doubles, Booleans, and Floats
    int landerState = 0;
    int hangArmLockCounter = 0;
    double offset = 0;
    double yAngle = 0;
    double time = System.currentTimeMillis();
    double bouncerPos = 0.0;
    double bicep = 17.5;
    double forearm = 16.0;
    double startingElbowPos = 0;
    double startingShoulderPos = 0;
    double speed = 1;
    boolean timeState = true;
    boolean fieldCentric = false;
    boolean yPressed = false;
    boolean isGettingOnLander = false;
    boolean firstTimeLander = true;
    boolean firstTimeRaisingArm = true;
    boolean zeroPowerBehavior = false;
    boolean isPressedX = false;
    boolean isPressedB = false;
    boolean isPressedY1Gamepad = false;
    boolean slowModeArm = false;
    boolean isFieldCentricButton = false;
    boolean armMode = false;
    boolean isRunningArm = false;
    boolean armMoved = false;
    boolean heldPositionLast = false;
    int heldElbowPos = 0;
    int heldShoulderPos = 0;
    float rightX;
    float leftX;
    float leftY;
    String angleDouble = "hi";
    boolean newGamepad2Pressed = true;

    //Robot Hardware
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx middleMotor;
    DcMotorEx middleMotor2;
    DcMotorEx hangArm;

    DcMotorEx shoulderMotor;
    DcMotorEx elbowMotor;
    DcMotorEx rotationMotor;
    Servo hangArmLock;
    Servo leftIntake;
    Servo rightIntake;
    Servo bouncer;

    Orientation angles;
    BNO055IMU imu;
    PIDFCoefficients pidStuff;

    public void init() {
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
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotor");
        middleMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor");
        middleMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor2");
        shoulderMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Shoulder Motor");
        elbowMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Elbow Motor");
        rotationMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Rotation Motor");
        leftIntake = hardwareMap.get(Servo.class, "Left Intake");
        rightIntake = hardwareMap.get(Servo.class, "Right Intake");
        bouncer = hardwareMap.get(Servo.class, "Bouncer");
        hangArm = (DcMotorEx) hardwareMap.get(DcMotor.class, "Hang Arm");
        hangArmLock = hardwareMap.servo.get("Hang Arm Lock");
        calculator = new HDriveFCCalc();
        armCalculator = new ArmCalculator(bicep, forearm);

        elbowMotor.setDirection(DcMotor.Direction.REVERSE);
        rotationMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor2.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pidStuff.p = 5;
        pidStuff.i = .4;
        pidStuff.d = 0;
        pidStuff.f = 10.2;
        pidStuff.algorithm = MotorControlAlgorithm.PIDF;
        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
        rightIntake.setDirection(Servo.Direction.REVERSE);
        leftIntake.setPosition(.5);
        rightIntake.setPosition(.5);
        bouncer.setPosition(0);
    }

    public void loop() {
        //Always Resets these
        telemetry.addData("getelbowmotor: ", armCalculator.getElbowMotorSpeed());
        telemetry.addData("Shoulder Motor", shoulderMotor.getPower());
        telemetry.addData("Ebow Motor", elbowMotor.getPower());
        telemetry.addData("Rotation Position", rotationMotor.getCurrentPosition());
        telemetry.addData("Shoulder Position", shoulderMotor.getCurrentPosition());
        telemetry.addData("Elbow Position", elbowMotor.getCurrentPosition());
        telemetry.addData("Zero Power Behavior", zeroPowerBehavior);
        telemetry.addData("Right Stick", gamepad2.right_stick_x);
        telemetry.addData("Elbow Pos", elbowMotor.getCurrentPosition());
        telemetry.addData("Shoulder pos", shoulderMotor.getCurrentPosition());
        telemetry.addData("Elbow Angle", elbowAngle((double) elbowMotor.getCurrentPosition()));
        telemetry.addData("Shoulder Angle", shoulderAngle((double) shoulderMotor.getCurrentPosition()));
        telemetry.addData("Arm Mode", armMode);
        telemetry.addData("Is Pressed X", isPressedX);
        telemetry.addData("Bouncer Pos", bouncer.getPosition());
        telemetry.update();
        armCalculator.calculateSpeed(gamepad2.left_stick_x, -gamepad2.right_stick_y, shoulderMotor, elbowMotor, shoulderAngle((double) shoulderMotor.getCurrentPosition()), elbowAngle((double) elbowMotor.getCurrentPosition()), telemetry);
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(gamepad2.y){
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setTargetPosition(3427);
            elbowMotor.setTargetPosition(-1125);
            shoulderMotor.setPower(1.0);
            elbowMotor.setPower(0.9);
            telemetry.addData("elbow pos", elbowMotor.getCurrentPosition());
            telemetry.addData("shoulder pos", shoulderMotor.getCurrentPosition());
            telemetry.update();
            armMoved = true;
            isRunningArm = true;
        }else{
            //releaseArm();
        }
        if (gamepad2.a) {
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setTargetPosition(1550);
            elbowMotor.setTargetPosition(1500);
            shoulderMotor.setPower(1.0);
            elbowMotor.setPower(0.9);
            telemetry.addData("elbow pos", elbowMotor.getCurrentPosition());
            telemetry.addData("shoulder pos", shoulderMotor.getCurrentPosition());
            telemetry.update();
            armMoved = true;
            isRunningArm = true;
            armMoved = true;
        }
        else if(!gamepad2.a && !gamepad2.y) {
            isRunningArm = false;
        }
        else {
            firstTimeRaisingArm = true;
            /*if(shoulderMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION || elbowMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }*/
        }

        if (gamepad2.x && isPressedX) {
            isPressedX = false;
            if (armMode) {
                armMode = false;
            } else {
                armMode = true;
            }
        } else if (!gamepad2.x) {
            isPressedX = true;
        }

        if (gamepad2.b && isPressedB) {
            if(slowModeArm) {
                slowModeArm = false;
            }
            else {
                slowModeArm = true;
            }
            isPressedB = false;

        }
        else if(!gamepad2.b) {
            isPressedB = true;
            slowModeArm = false;
        }
        if (armMode) {
            //elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if ((gamepad2.left_stick_x != 0 || gamepad2.left_stick_y != 0 || gamepad2.right_stick_x != 0)) {
                zeroPowerBehavior = false;
                armMoved = true;
                releaseArm();
                if (gamepad2.b && slowModeArm ) {
                    rotationMotor.setPower(.2 * gamepad2.right_stick_x);
                    elbowMotor.setPower(.2 * -gamepad2.left_stick_y);
                    shoulderMotor.setPower(.2 * gamepad2.left_stick_x);

                }
                else {
                    rotationMotor.setPower(.5 * gamepad2.right_stick_x);
                    elbowMotor.setPower(.5 * -gamepad2.left_stick_y);
                    shoulderMotor.setPower(.5 * gamepad2.left_stick_x);
                }
            }
        } else if (armMode == false) {
            if (gamepad2.x && isPressedX) {
                isPressedX = false;
                if (armMode) {
                    armMode = false;
                } else {
                    armMode = true;
                }
            } else if (!gamepad2.x) {
                isPressedX = true;
            }
            if (gamepad2.b && slowModeArm && (gamepad2.left_stick_x != 0 || gamepad2.left_stick_y != 0 || gamepad2.right_stick_x != 0)) {
                releaseArm();
                //elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rotationMotor.setPower(.3 * gamepad2.right_stick_x);
                elbowMotor.setPower(.2 * armCalculator.getElbowMotorSpeed());
                shoulderMotor.setPower(.2 * armCalculator.getShoulderMotorSpeed());
                zeroPowerBehavior = false;
                armMoved = true;
            } else if (gamepad2.left_stick_x == 0 && gamepad2.left_stick_y == 0 && gamepad2.right_stick_x == 0) {
               /* rotationMotor.setPower(0);
                elbowMotor.setPower(0);
                shoulderMotor.setPower(0);
                //elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbowMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                shoulderMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                //elbowMotor.setTargetPosition(elbowMotor.getCurrentPosition());
                //shoulderMotor.setTargetPosition(shoulderMotor.getCurrentPosition());
                zeroPowerBehavior = true;*/
            } else {
                releaseArm();
                //elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rotationMotor.setPower(gamepad2.right_stick_x);
                elbowMotor.setPower(.4 * armCalculator.getElbowMotorSpeed());
                shoulderMotor.setPower(.4 * armCalculator.getShoulderMotorSpeed());
                zeroPowerBehavior = false;
                armMoved = true;
            }
        }
        if (gamepad2.left_bumper) {
            leftIntake.setPosition(.05);
        } else {
            leftIntake.setPosition(0.5 + (gamepad2.left_trigger / 2.3));
        }
        if (gamepad2.right_bumper) {
            rightIntake.setPosition(.05);
        } else {
            rightIntake.setPosition(0.5 + (gamepad2.right_trigger / 2.3));

        }
        if (gamepad2.dpad_up) {
            bouncerPos = .3; //.58
        } else if (gamepad2.dpad_down) {
            bouncerPos = 0;//.73fr
        }
        bouncer.setPosition(bouncerPos);


        if (gamepad1.left_bumper) {
            getOnLander();
            isGettingOnLander = true;
        } else {
            firstTimeLander = true;
            isGettingOnLander = false;
        }


        if (isGettingOnLander == false) {
            if (yPressed == false) {
                leftX = gamepad1.left_stick_x;
                leftY = gamepad1.left_stick_y;
            }
            if (gamepad1.a != true && yPressed == false) {
                rightX = gamepad1.right_stick_x;
            }
            if (gamepad1.b == true) {
                offset = Double.parseDouble(angleDouble) + 180;
                offset = -offset;
            }


            if (gamepad1.x && isFieldCentricButton) {
                isFieldCentricButton = false;
                if (fieldCentric) {
                    fieldCentric = false;
                } else {
                    fieldCentric = true;
                }
            } else if (!gamepad1.x) {
                isFieldCentricButton = true;
            }



            if (fieldCentric) {
                calculator.calculateMovement(leftX, leftY, rightX, Double.parseDouble(angleDouble) + offset);
            } else {
                calculator.calculateMovement(leftX, leftY, rightX, 180);
            }
            if(gamepad1.left_stick_button && isPressedY1Gamepad) {
                if(speed == 1) {
                    speed = .5;
                }
                else {
                    speed = 1;
                }
                isPressedY1Gamepad = false;
            }
            else if(!gamepad1.left_stick_button) {
                isPressedY1Gamepad = true;
            }
            leftMotor.setPower(speed * calculator.getLeftDrive());
            rightMotor.setPower(speed* calculator.getRightDrive());
            middleMotor.setPower(speed * calculator.getMiddleDrive());
            middleMotor2.setPower(speed * calculator.getMiddleDrive());
        }
        if(!armMoved){
            holdArmPosition();
        }
        else{
            heldPositionLast = false;
        }
        armMoved = false;


    }

    public void getOnLander() {
        telemetry.addData("State", landerState);
        telemetry.addData("Middle Motor", middleMotor.getCurrentPosition());
        telemetry.addData("Hang Arm", hangArm.getCurrentPosition());
        telemetry.addData("Intake Pos", bouncerPos);
        telemetry.update();
        if (firstTimeLander) {
            landerState = 0;
            hangArmLockCounter = 0;
        }
        firstTimeLander = false;

        if (landerState == 0) {
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor.setPower(-.2);
            middleMotor2.setPower(-.2);
            landerState = 1;
        } else if (landerState == 1) {
            if (gamepad1.right_bumper == true) {
                middleMotor.setPower(0);
                middleMotor2.setPower(0);
                landerState = 2;
            }
        } else if (landerState == 2) {
            hangArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hangArm.setTargetPosition(2000);
            hangArm.setPower(.5);
            landerState = 3;
        } else if (landerState == 3) {
            if (!hangArm.isBusy()) {
                hangArm.setPower(0);
                hangArm.setTargetPosition(hangArm.getCurrentPosition());
                hangArmLock.setPosition(.59);
                landerState = 4;
            }
        } else if (landerState == 4) {
            hangArmLockCounter++;
            hangArm.setTargetPosition(hangArm.getCurrentPosition());
            telemetry.addData("Counter", hangArmLockCounter);
            telemetry.update();
            if (hangArmLockCounter > 30) {
                landerState = 5;
            }
        } else if (landerState == 5) {
            hangArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hangArm.setPower(0);
            telemetry.addLine("Hang Complete");
            telemetry.update();
            landerState++;
        } else if(landerState == 6) {
            hangArm.setPower(.2);
        }
    }

    public void runToGivenPos(double elbowPos, double shoulderPos, double elbowPower, double shoulderPower) {
        boolean isChanged = false;
        if (firstTimeRaisingArm) {
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setTargetPosition((int) shoulderPos);
            elbowMotor.setTargetPosition((int) elbowPos);
            firstTimeRaisingArm = false;
        }
        shoulderMotor.setPower(shoulderPower);
        elbowMotor.setPower(elbowPower);
        telemetry.addData("Elbow Pos", elbowMotor.getCurrentPosition());
        telemetry.addData("Shoulder Pos", shoulderMotor.getCurrentPosition());
        telemetry.addData("Elbow Target", elbowPos);
        telemetry.addData("Shoulder Pos", shoulderPos);
        telemetry.update();
        if (elbowMotor.getCurrentPosition() > elbowPos) {
                elbowMotor.setPower(0);
            elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (shoulderMotor.getCurrentPosition() > shoulderPos) {
                shoulderMotor.setPower(0);
            shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public double elbowAngle(double currentPos) {
        double finalPos = 90 - shoulderAngle((double) shoulderMotor.getCurrentPosition()) + 45 + ((currentPos + (2300 - 1576)) / (2300 * 4)) * 360;
        return finalPos;
    }

    public double shoulderAngle(double currentPos) {
        double finalPos = ((-currentPos + 4330) / (2350 * 4)) * 360;
        return finalPos;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public void holdArmPosition(){
        if(!heldPositionLast){
            heldElbowPos = elbowMotor.getCurrentPosition();
            heldShoulderPos = shoulderMotor.getCurrentPosition();
            heldPositionLast = true;
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        elbowMotor.setPower(.1);
        shoulderMotor.setPower(.1);
        rotationMotor.setPower(0);
        elbowMotor.setTargetPosition(heldElbowPos);
        shoulderMotor.setTargetPosition(heldShoulderPos);

    }
    public void releaseArm(){
        if(shoulderMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION || elbowMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    /*public void stop(){
        super.stop();
        hangArmLock.setPosition(.62);
    }*/
}

