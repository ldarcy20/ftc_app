package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

@TeleOp(name = "Fresh Boi", group = "HDrive")
public class MrFresh extends OpMode {
    HDriveFCCalc calculator;
    ArmCalculator armCalculator;

    //Ints, Doubles, Booleans, and Floats
    int landerState = 0;
    int hangArmLockCounter = 0;
    double offset = 0;
    double yAngle = 0;
    double time = System.currentTimeMillis();
    double bouncerPos = 0.0;
    double bicep = 18;
    double forearm = 17.25;
    double startingElbowPos = 0;
    double startingShoulderPos = 0;
    double speed = 1;
    double startingAngle = 0;
    double endingAngle = 0;
    double angleError = 0;
    double sideChange = 0;
    double sidePower = 0;
    double offsetSide = 0;
    double offsetMid = 0;
    double craterState = 0;

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
    boolean sideMoved = false;
    boolean heldPositionLast = false;
    boolean heldSidePositionLast = false;
    boolean firstTimeMovingLander = true;
    boolean isMovingTowardsLander = true;
    boolean firstTimeStopedLander = false;
    boolean firstTimeCrater = true;
    boolean firstTimeLanderMove = true;
    boolean firstTimeScoreLander = true;
    int heldElbowPos = 0;
    int heldSidePosLeft = 0;
    int heldSidePosRight = 0;
    int heldShoulderPos = 0;
    float rightX;
    double leftX;
    double leftY;
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
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        pidStuff.i = 4;
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
        hangArmLock.setPosition(.35);
    }
    public void loop() {
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Always Resets these
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = ExcessStuff.formatAngle(angles.angleUnit, angles.firstAngle);
        armCalculator.calculateSpeed(gamepad2.left_stick_x, -gamepad2.right_stick_y, shoulderMotor, elbowMotor, ExcessStuff.shoulderAngle((double) shoulderMotor.getCurrentPosition()), ExcessStuff.elbowAngle((double) elbowMotor.getCurrentPosition()), telemetry);
        telemetry.addData("Left Stick",calculator.isStupid());
        telemetry.addData("Left Power", calculator.getLeftDrive());
        telemetry.addData("Right Actual", calculator.getRightDrive());
        telemetry.addData("Left Motor", leftMotor.getCurrentPosition());
        telemetry.addData("Middle Motor", middleMotor.getCurrentPosition());
        telemetry.addData("Hang Motor", hangArm.getCurrentPosition());
        telemetry.addData("Elbow Motor", elbowMotor.getCurrentPosition());
        telemetry.addData("Rotation Motor", rotationMotor.getCurrentPosition());
        telemetry.addData("Shoulder Motor", shoulderMotor.getCurrentPosition());
        //telemetry.addData("Shoulder Angle", shoulderAngle(shoulderMotor.getCurrentPosition()));
        //telemetry.addData("Elbow Angle", elbowAngle(elbowMotor.getCurrentPosition()));
        telemetry.addData("Arm Mode", armMode);
        telemetry.addData("Arm Moved", armMoved);
        //telemetry.addData("Elbow Angle", elbowAngle(elbowMotor.getCurrentPosition()));
        //telemetry.addData("Shoulder Angle", shoulderAngle(shoulderMotor.getCurrentPosition()));
        telemetry.addData("Elbow Vel", armCalculator.getElbowMotorSpeed());
        telemetry.addData("Shoulder Vel", armCalculator.getShoulderMotorSpeed());
        telemetry.update();

        dpadCheck();
        checkArmMovement();
        checkFieldCentricAndSlowMode();
        checkIntake();
        gettingOnTheLander();
        moveTheBase();
        ScoringAndCollectingButtons();




        if (!armMoved) {
            holdArmPosition();
        } else {
            heldPositionLast = false;
        }
        /*telemetry.addData("Side Moved", sideMoved);
        if(!sideMoved) {
            holdSideMotorsPosition();
        }
        else {
            heldSidePositionLast = false;
        }*/
        armMoved = false;
        sideMoved = false;
    }
    public void dpadCheck() {
        if (gamepad2.dpad_up) {
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setTargetPosition(50);
            elbowMotor.setTargetPosition(50);
            rotationMotor.setTargetPosition(-25);
            shoulderMotor.setPower(.5);
            elbowMotor.setPower(0.5);
            rotationMotor.setPower(.3);
            telemetry.addData("elbow pos", elbowMotor.getCurrentPosition());
            telemetry.addData("shoulder pos", shoulderMotor.getCurrentPosition());
            telemetry.addData("rotation Pos", rotationMotor.getCurrentPosition());
            telemetry.update();
            armMoved = true;
            isRunningArm = true;
        }
        if (gamepad2.dpad_down) {
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setTargetPosition(3982);
            elbowMotor.setTargetPosition(-368);
            shoulderMotor.setPower(.7);
            elbowMotor.setPower(0.7);
            telemetry.addData("elbow pos", elbowMotor.getCurrentPosition());
            telemetry.addData("shoulder pos", shoulderMotor.getCurrentPosition());
            telemetry.update();
            armMoved = true;
            isRunningArm = true;
        }
    }

    public void checkArmMovement() {
        if (armMode) {
            if ((gamepad2.left_stick_x != 0 || gamepad2.left_stick_y != 0 || gamepad2.right_stick_x != 0)) {
                zeroPowerBehavior = false;
                armMoved = true;
                releaseArm();
                if (gamepad2.b && slowModeArm) {
                    rotationMotor.setPower(.2 * gamepad2.right_stick_x);
                    elbowMotor.setPower(.2 * -gamepad2.left_stick_y);
                    shoulderMotor.setPower(.2 * gamepad2.left_stick_x);

                } else {
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
                rotationMotor.setPower(.3 * gamepad2.right_stick_x);
                elbowMotor.setPower(.2 * armCalculator.getElbowMotorSpeed());
                shoulderMotor.setPower(.2 * armCalculator.getShoulderMotorSpeed());
                zeroPowerBehavior = false;
                armMoved = true;
            } else if ((gamepad2.left_stick_x != 0 || gamepad2.left_stick_y != 0 || gamepad2.right_stick_x != 0)) {
                releaseArm();
                rotationMotor.setPower(gamepad2.right_stick_x);
                elbowMotor.setPower(.4 * armCalculator.getElbowMotorSpeed());
                shoulderMotor.setPower(.4 * armCalculator.getShoulderMotorSpeed());
                zeroPowerBehavior = false;
                armMoved = true;
            }
        }
    }
    public void checkFieldCentricAndSlowMode() {
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
            if (slowModeArm) {
                slowModeArm = false;
            } else {
                slowModeArm = true;
            }
            isPressedB = false;

        } else if (!gamepad2.b) {
            isPressedB = true;
            slowModeArm = false;
        }
    }
    public void checkIntake() {
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
            bouncerPos = 0;//.73
        }
        bouncer.setPosition(bouncerPos);
    }
    public void gettingOnTheLander() {
        if (gamepad1.left_bumper) {
            getOnLander();
            isGettingOnLander = true;
        } else {
            firstTimeLander = true;
            isGettingOnLander = false;
        }
        if (gamepad1.right_trigger == 1) {
            goToLander();
            isMovingTowardsLander = true;
        } else {
            if (isMovingTowardsLander) {
                pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                pidStuff.p = 0;
                pidStuff.i = 0;
                pidStuff.d = 0;
                pidStuff.f = 0;
                pidStuff.algorithm = MotorControlAlgorithm.PIDF;
                leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
                rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
                middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
                middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                middleMotor.setPower(0);
                middleMotor2.setPower(0);
                pidStuff = leftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                pidStuff.p = 5;
                pidStuff.i = 4;
                pidStuff.d = 0;
                pidStuff.f = 10.2;
                pidStuff.algorithm = MotorControlAlgorithm.PIDF;
                leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
                rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
                middleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
                middleMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidStuff);
            }
            isMovingTowardsLander = false;
            firstTimeMovingLander = true;
            firstTimeStopedLander = true;
        }
    }
    public void moveTheBase() {
        if (isGettingOnLander == false && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_down && !gamepad1.dpad_up && !isMovingTowardsLander && !gamepad2.y) {
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
            if (gamepad1.left_stick_button && isPressedY1Gamepad) {
                if (speed == 1) {
                    speed = .5;
                } else {
                    speed = 1;
                }
                isPressedY1Gamepad = false;
            } else if (!gamepad1.left_stick_button) {
                isPressedY1Gamepad = true;
            }
            if (calculator.getLeftDrive() != 0 || calculator.getRightDrive() != 0 || calculator.getMiddleDrive() != 0) {
                sideMoved = true;
            }
            leftMotor.setPower(speed * calculator.getLeftDrive());
            rightMotor.setPower(speed * calculator.getRightDrive());
            middleMotor.setPower(speed * calculator.getMiddleDrive());
            middleMotor2.setPower(speed * calculator.getMiddleDrive());
        }
    }
    public void holdArmPosition(){
        if(!heldPositionLast){
            heldElbowPos = elbowMotor.getCurrentPosition();
            heldShoulderPos = shoulderMotor.getCurrentPosition();
            heldPositionLast = true;
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        elbowMotor.setPower(.2);
        shoulderMotor.setPower(.2);
        rotationMotor.setPower(0);
        elbowMotor.setTargetPosition(heldElbowPos);
        shoulderMotor.setTargetPosition(heldShoulderPos);

    }
    public void releaseArm(){
        if(shoulderMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION || elbowMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION || rotationMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION); {
            shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void goToLander() {
        if(firstTimeMovingLander) {
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middleMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setTargetPosition(0 + (int)offsetSide); //-2000
            rightMotor.setTargetPosition(0 + (int)offsetSide);//-2000
            middleMotor.setTargetPosition(0 + (int)offsetMid);//-4000
            middleMotor2.setTargetPosition(0 + (int)offsetMid);//-4000
            leftMotor.setPower(.4);
            rightMotor.setPower(.4);
            middleMotor.setPower(.9);
            middleMotor2.setPower(.9);
            firstTimeMovingLander = false;
            angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            angleDouble = ExcessStuff.formatAngle(angles.angleUnit, angles.firstAngle);
            if(leftMotor.getCurrentPosition() > 0 + (int)offsetSide) {
                sidePower = -.4;
            }
            else if(leftMotor.getCurrentPosition() < 0 + (int)offsetSide) {
                sidePower = .4;
            }
        }
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = ExcessStuff.formatAngle(angles.angleUnit, angles.firstAngle);
        endingAngle = Double.parseDouble(angleDouble);
        angleError = endingAngle - startingAngle;
        sideChange = (angleError)/100;
        if(leftMotor.isBusy() && rightMotor.isBusy()) {
            leftMotor.setPower(sidePower + sideChange);
            rightMotor.setPower(sidePower - sideChange);
        }
        if(!rightMotor.isBusy() || !leftMotor.isBusy()) {
            if(rightMotor.getMode() == RUN_TO_POSITION || leftMotor.getMode() == RUN_TO_POSITION) {
                leftMotor.setMode(RUN_USING_ENCODER);
                rightMotor.setMode(RUN_USING_ENCODER);
            }
            leftMotor.setPower(sideChange);
            rightMotor.setPower(-sideChange);
        }
        if(!leftMotor.isBusy() && !rightMotor.isBusy() && !middleMotor.isBusy() && !middleMotor2.isBusy()){
            if(firstTimeStopedLander) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                middleMotor.setPower(0);
                middleMotor2.setPower(0);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                firstTimeStopedLander = false;
            }
        }
        sideMoved = true;
    }
    public void getOnLander() {
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
            hangArm.setTargetPosition(-2200);
            hangArm.setPower(-.5);
            landerState = 3;
        } else if (landerState == 3) {
            if (!hangArm.isBusy()) {
                hangArm.setPower(.4);
                hangArm.setTargetPosition(hangArm.getCurrentPosition());
                hangArmLock.setPosition(.59);
                landerState = 4;
            }
        } else if (landerState == 4) {
            hangArmLockCounter++;
            hangArm.setTargetPosition(hangArm.getCurrentPosition());
            if (hangArmLockCounter > 30) {
                hangArmLock.setPosition(.55);
                landerState = 5;
            }
        } else if (landerState == 5) {
            hangArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hangArm.setPower(0);
            landerState++;
        } else if (landerState == 6) {
            hangArm.setPower(.2);
            hangArmLock.setPosition(.59);
        }
    }
    public void ScoringAndCollectingButtons() {
        if(gamepad1.y) {
            collectFromCrater();
        } else if(gamepad1.a) {
            scoreInLander();
        }
        else {
            craterState = 0;
            landerState = 0;
        }
    }
    public void scoreInLander() {
        if (landerState == 0) {
            rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotationMotor.setTargetPosition(0);
            shoulderMotor.setTargetPosition(0);
            elbowMotor.setTargetPosition(0);
            leftMotor.setTargetPosition(0);
            rightMotor.setTargetPosition(0);
            rotationMotor.setPower(.2);
            shoulderMotor.setPower(.2);
            elbowMotor.setPower(.2);
            landerState = 1;
        }
        if(landerState == 1) {
            if(!shoulderMotor.isBusy()) {
                landerState = 2;
            }
        }
        if(landerState == 2) {
            leftMotor.setPower(.2);
            rightMotor.setPower(.2);
        }
    }
    public void collectFromCrater() {
        if(craterState == 0) {
            rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotationMotor.setTargetPosition(-525);
            shoulderMotor.setTargetPosition(3807);
            elbowMotor.setTargetPosition(-380);
            leftMotor.setTargetPosition(0);
            rightMotor.setTargetPosition(0);
            leftMotor.setPower(.2);
            rightMotor.setPower(.2);;
            craterState = 1;
        }
        if(craterState == 1) {
            if(!leftMotor.isBusy() && !rightMotor.isBusy()) {
                craterState = 2;
            }
        }
        if(craterState == 2) {
            rotationMotor.setPower(.2);
            shoulderMotor.setPower(.2);
            elbowMotor.setPower(.2);
        }
    }
}
