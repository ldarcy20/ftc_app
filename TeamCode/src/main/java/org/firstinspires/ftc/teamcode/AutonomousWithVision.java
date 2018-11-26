package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
import android.util.Range;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

/**
 * Created by Luke on 9/25/2016.
 */
@Autonomous(name= "AutoBoto ", group = "HDrive")
public class AutonomousWithVision extends LinearOpMode {
    VuforiaLocalizer vuforia;
    BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();// Use a Pushbot's hardware
    static final double     COUNTS_PER_MOTOR_REV    = 1150 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4; ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor middleMotor;
    DcMotor glyphMotor;
    DcMotor RelicArm;
    DcMotor middleMotor2;
    Servo claw1;
    Servo claw2;
    Servo RelicClaw;
    Servo RelicClaw2;
    AutoCalculator calculator;
    Orientation angles;
    String angleDouble = "hi";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AUNOqaX/////AAABmRYrfwXgf0eBiFui4bMgqFOA9l1KyDSRCEC94Cy8lAqVsIBXI7SCxQxBhxloFpJ43BYXjodIylbiXiArCEhYbdiVvgI7iXO35pCSMnMLhSwZ7+YnogjWO4wfJEe+WNGEV3sx1sI9Y14W45etXXpo7KzyGC5Eq8WTXkxSiIe8bnkSn3aZ+L1CFgIyk6fDajfxvyYWaZBlxQDM8fxIyBfthI0EBtfplBFrJLxqq8tgUEDBPiVlYcUuspKPk5ew1u4cHDJvtxC1UkIsXpABs7kGkSvh5/nAp02Hd7yWlw3UUlOGKDlOit2guJZEC/iSJqR5oQsx/nqLJakwCd6Z1NZZFAtZ2u8Gc4NtDpw+LW4o6uMD";
    private TFObjectDetector tfod;
    @Override public void runOpMode() throws InterruptedException {
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initVuforia();
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AUNOqaX/////AAABmRYrfwXgf0eBiFui4bMgqFOA9l1KyDSRCEC94Cy8lAqVsIBXI7SCxQxBhxloFpJ43BYXjodIylbiXiArCEhYbdiVvgI7iXO35pCSMnMLhSwZ7+YnogjWO4wfJEe+WNGEV3sx1sI9Y14W45etXXpo7KzyGC5Eq8WTXkxSiIe8bnkSn3aZ+L1CFgIyk6fDajfxvyYWaZBlxQDM8fxIyBfthI0EBtfplBFrJLxqq8tgUEDBPiVlYcUuspKPk5ew1u4cHDJvtxC1UkIsXpABs7kGkSvh5/nAp02Hd7yWlw3UUlOGKDlOit2guJZEC/iSJqR5oQsx/nqLJakwCd6Z1NZZFAtZ2u8Gc4NtDpw+LW4o6uMD";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        telemetry.addData(">", "created by the best programmer adam");
        telemetry.addData(">", "Press play to start");
        telemetry.update();*/

        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);

        BNO055IMU.Parameters parameters3 = new BNO055IMU.Parameters();
        parameters3.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters3.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters3.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters3.loggingEnabled = true;
        parameters3.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters3);
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        leftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rightMotor");
        middleMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor");
        middleMotor2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "middleMotor2");
        glyphMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Hirsh is very dumb");
        claw1 = hardwareMap.servo.get("claw1");
        claw2 = hardwareMap.servo.get("claw2");
        RelicClaw = hardwareMap.get(Servo.class, "Relic Claw");
        RelicClaw2 = hardwareMap.get(Servo.class, "Relic Claw 2");
        RelicArm = hardwareMap.get(DcMotor.class, "Relic Arm");

        /*
         * Initialize the drive system variables.
         * The init
         * () method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders for awesome reason");    //
        telemetry.update();
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        glyphMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition(),
                middleMotor.getCurrentPosition());
        telemetry.update();
        rightMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        middleMotor.setDirection(DcMotor.Direction.FORWARD);
        middleMotor2.setDirection(DcMotor.Direction.FORWARD);
        calculator = new AutoCalculator();

        waitForStart();
        tfod.activate();
        int objectsFound = 0;
        int position = 0;
        while(objectsFound != 3) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    objectsFound = updatedRecognitions.size();
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                position = 1;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                position = 3;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                position = 2;
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }
        if(position == 1) {
            telemetry.addLine("Should be Left");
            encoderDriveBoth(.5, .5, 20, 20, 20, 60);
        }
        else if(position == 2) {
            telemetry.addLine("Should be Middle");
            encoderDrive(.5,20,20,60);
        }
        else if(position == 3) {
            telemetry.addLine("Should be Right");
            encoderDriveBoth(.5,-.5,-20,20,20,60);
        }
        else {
            telemetry.addLine("Yea you broke something lmao");
            telemetry.update();
        }


        //calculator.calculateMovement(180, 180);
        leftMotor.setPower(.5 * calculator.getLeftDrive());
        rightMotor.setPower(.5 * calculator.getRightDrive());
        middleMotor.setPower(.5 * calculator.getMiddleDrive());
        Thread.sleep(5000);
        telemetry.addData("left Move", calculator.leftMove);
        telemetry.addData("right move", calculator.rightMove);
        telemetry.update();
        //encoderDrive(.3,36,36,60);
        System.exit(0);
    }
    public void encoderDriveMiddle(double speed,//int leftDistance, int rightDistance,
                                   double middleInches,
                                   double timeoutS) throws InterruptedException {

        int newMiddleTarget = 0;
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newMiddleTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((middleInches) * COUNTS_PER_INCH);
            middleMotor.setTargetPosition(newMiddleTarget);
            middleMotor2.setTargetPosition(newMiddleTarget);
            // Turn On RUN_TO_POSITION
            middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middleMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            middleMotor.setPower(speed);
            middleMotor2.setPower(speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (middleMotor.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1", "Running to %7d", newMiddleTarget);
                //telemetry.addData("Path2", "Running at %7d",
                //        middleMotor.getCurrentPosition());
                //telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            middleMotor.setPower(0);
            middleMotor2.setPower(0);

            // Turn off RUN_TO_POSITION
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDrive(double speed,//int leftDistance, int rightDistance,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {



        int newLeftTarget = 0;
        int newRightTarget = 0;
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = /*robot.leftMotor.getCurrentPosition() + */(int)((leftInches) * COUNTS_PER_INCH);
            newRightTarget = /*robot.rightMotor.getCurrentPosition() + */(int)((rightInches) * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Encoder Left", leftMotor.getCurrentPosition());
                telemetry.addData("Encoder Right", rightMotor.getCurrentPosition());
                //leftMotor.getCurrentPosition(),
                //rightMotor.getCurrentPosition();

                 telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveMotionProfileing(double speed,//int leftDistance, int rightDistance,
                                             double leftInches, double rightInches,
                                             double timeoutS) throws InterruptedException {



        int newLeftTarget = 0;
        int newRightTarget = 0;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = /*robot.leftMotor.getCurrentPosition() + */(int)((leftInches) * COUNTS_PER_INCH);
            newRightTarget = /*robot.rightMotor.getCurrentPosition() + */(int)((rightInches) * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);
            int speedWeWant = 0;
            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy() )) {
                if(leftMotor.getCurrentPosition() == (newLeftTarget/3)*2) {

                }
                // Display it for the driver.
                telemetry.addData("Encoder Left", leftMotor.getCurrentPosition());
                telemetry.addData("Encoder Right", rightMotor.getCurrentPosition());
                //leftMotor.getCurrentPosition(),
                //rightMotor.getCurrentPosition();

                // telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveBoth(double speed, double speedMiddle,//int leftDistance, int rightDistance,
                                 double middleInches, double leftInches, double rightInches,
                                 double timeoutS) throws InterruptedException {

        int newMiddleTarget = 0;
        int newLeftTarget = 0;
        int newRightTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newMiddleTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((middleInches) * COUNTS_PER_INCH);
            newLeftTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((rightInches) * COUNTS_PER_INCH);
            newRightTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((leftInches) * COUNTS_PER_INCH);
            middleMotor.setTargetPosition(newMiddleTarget);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            middleMotor2.setTargetPosition(newMiddleTarget);
            // Turn On RUN_TO_POSITION
            middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            middleMotor.setPower(speedMiddle);
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);
            middleMotor2.setPower(speedMiddle);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (middleMotor.isBusy() && leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1", "Running to %7d", newMiddleTarget);
                //telemetry.addData("Path2", "Running at %7d",
                //        middleMotor.getCurrentPosition());
                //telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            middleMotor.setPower(0);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            middleMotor2.setPower(0);

            // Turn off RUN_TO_POSITION
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            //  sleep(250);   // optional pause after each move
        }
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void encoderDriveAngle(double speed,//int leftDistance, int rightDistance,
                                  double inches, double timeoutS, double moveAngle) throws InterruptedException {

        // add gyro crap
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        double robotAngle =  Double.parseDouble(angleDouble);



        int newMiddleTarget = 0;
        int newLeftTarget = 0;
        int newRightTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            double theta = moveAngle/360 * 2 * Math.PI;
            double radianRobotAngle = -robotAngle/360 * 2 * Math.PI;
            double tempSideVal = Math.sin(theta) * Math.cos(radianRobotAngle) - Math.cos(theta) * Math.sin(radianRobotAngle);
            double tempMiddleVal = Math.cos(theta) * Math.cos(-radianRobotAngle) - Math.sin(theta) * Math.sin(-radianRobotAngle);
            int sideInches = (int) (inches* tempSideVal);
            int middleInches = (int) (inches* tempMiddleVal);
            double scale =  middleInches/sideInches;
            double speedMiddle = speed * scale;

            if (speedMiddle > 1) {
                speed = speed * (1 / speedMiddle);
                speedMiddle = 1;
            }

            // Determine new target position, and pass to motor controller
            newMiddleTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((middleInches) * COUNTS_PER_INCH);
            newLeftTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((sideInches) * COUNTS_PER_INCH);
            newRightTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((sideInches) * COUNTS_PER_INCH);
            middleMotor.setTargetPosition(newMiddleTarget);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            middleMotor2.setTargetPosition(newMiddleTarget);
            // Turn On RUN_TO_POSITION
            middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middleMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            middleMotor.setPower(speedMiddle);
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);
            middleMotor2.setPower(speedMiddle);


            // keep looping while we are still active, and there is time left, and both motors are running.
            /*while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (middleMotor.isBusy() && leftMotor.isBusy() && rightMotor.isBusy()) && middleMotor2.isBusy()) {*/
            while(leftMotor.getCurrentPosition() < newLeftTarget && middleMotor.getCurrentPosition() < newMiddleTarget) {

                // Display it for the driver.
                telemetry.addData("Path1" , leftMotor.getCurrentPosition());
                telemetry.addData("Path2",
                        middleMotor.getCurrentPosition());
                telemetry.addData("Left Power",leftMotor.getPower());
                telemetry.addData("Middle Power", middleMotor.getPower());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            middleMotor.setPower(0);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            middleMotor2.setPower(0);

            // Turn off RUN_TO_POSITION
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            //  sleep(250);   // optional pause after each move
        }
    }


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
