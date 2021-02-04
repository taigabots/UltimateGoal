package org.firstinspires.ftc.teamcode.vision;



import android.os.IBinder;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class WobbleAutoFast extends LinearOpMode
{

    BNO055IMU               imu;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;
    public DcMotor LeftFront  = null;
    public DcMotor LeftRear   = null;
    public DcMotor RightFront = null;
    public DcMotor RightRear  = null;
    public DcMotor Shooter    = null;
    public Servo   ShootAngle = null;
    public Servo   ShooterArm = null;
    public DcMotor Intake     = null;
    public DcMotor WobbleArm  = null;
    public Servo   WobbleGrab = null;
    double WHEEL_CIRCUMFERENCE = 3.78;
    double ENCODER_TICKS_PER_ROTATION = 537.6;



    @Override
    public void runOpMode()
    {

//------------------------------PhoneHardWareMap--------------------------------------------------\\

        LeftRear = hardwareMap.dcMotor.get(" BackLeft    ");
        LeftFront = hardwareMap.dcMotor.get(" FrontLeft   ");
        RightFront = hardwareMap.dcMotor.get(" FrontRight  ");
        RightRear = hardwareMap.dcMotor.get(" BackRight   ");
        Intake = hardwareMap.dcMotor.get("Intake      ");
        Shooter = hardwareMap.dcMotor.get("Shooter     ");
        WobbleArm = hardwareMap.dcMotor.get("WobbleArm   ");
        ShooterArm = hardwareMap.servo.get("ShooterArm  ");
        ShootAngle = hardwareMap.servo.get("ShooterAngle");
        WobbleGrab = hardwareMap.servo.get("WobbleGrab  ");

//------------------------------Direction---------------------------------------------------------\\
        //Reverse spins motors to the right Forward spins motors to the left
        LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftRear  .setDirection (DcMotorSimple.Direction.FORWARD);
        RightFront.setDirection (DcMotorSimple.Direction.REVERSE);
        RightRear .setDirection (DcMotorSimple.Direction.REVERSE);
        Shooter   .setDirection (DcMotorSimple.Direction.REVERSE);
        Intake    .setDirection (DcMotorSimple.Direction.REVERSE);
        WobbleArm .setDirection (DcMotorSimple.Direction.REVERSE);

//------------------------------Encoder---------------------------------------------------------\\


        LeftFront .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRear  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightRear .setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftFront  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter    .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        //Dont look unless broken
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                //                                                         ^^ camera orintation
            }
        });

        waitForStart();

        resetAngle();

        /*
        while (opModeIsActive()){
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
            sleep(50);
        }
        */
        //^^ uncomment the peice to see the analisis in real time

        sleep(1000);

        if (pipeline.getAnalysis() > pipeline.FOUR_RING_THRESHOLD) {

            webcam.stopStreaming();
            telemetry.addData("square", "Far");
            telemetry.addData("Ring", "FOUR");
            telemetry.update();


            ShooterArm.setPosition(0.259);
            ShootAngle.setPosition(0.775);
            WobbleGrab.setPosition(0.38);
            // ^^ sets up servos for run
            Drive(55, -.75);
            // ^^ drives past rings
            Intake.setPower(1);
            Strafe(20, -.75);
            Shooter.setPower(1);
            ShootAngle.setPosition(.78);
            ShooterArm.setPosition(.259);
            // ^^ strafe to line up ring shots
            Intake.setPower(0);
            Drive(2, .75);
            // ^^ drives so robot straight
            Shoot(3, 1, .79);
            Shooter.setPower(0);
            ShooterArm.setPosition(.259);
            // ^^ shoots preloaded 3
            Intake.setPower(.75);
            ShootAngle.setPosition(1);
            // ^^ PREPARES TO INTAKE
//---------------------- ^ GOOD DO NOT CHANGE ^ --------------------------------------------------\\
            // ^^ shots 3 rings shoots the saplings into the high goal PRETTY COOL
            // ^^ Strafe For the 4 stack so it can eat up them yummy yummy saplings
            Drive(8, .25);
            Shooter.setPower(1);
            sleep(1000);
            // ^^ Intake the 4 stack ony 3 sometimes 2 cus intake garbo
            ShootAngle.setPosition(.78);
            // ^^ strafe to shoot
            Shoot(3, 1, .78);
            // ^^ shoots 6
            Shooter.setPower(0);
            // ^^ SHOOTS 3 MORE EPIC GAMER 36 points if gets 3
            ShootAngle.setPosition(1);
            Intake.setPower(.75);
            Drive(7, .25);
            Shooter.setPower(1);
            sleep(550);
            ShootAngle.setPosition(.785);
            sleep(250);
            Intake.setPower(0);
            ShooterArm.setPosition(.259);
            sleep(250);
            ShooterArm.setPosition(0);
            sleep(250);
            ShooterArm.setPosition(.259);
            sleep(250);
            ShooterArm.setPosition(0);
            sleep(500);
            Shooter.setPower(0);
            ShootAngle.setPosition(1);
            Drive(10, -1);
            Strafe(35, -.5);
            // ^^ strafe
            Drive(44, -1);
            // ^^ STUFF
            WobbleArm.setPower(-.75);
            sleep(1500);
            WobbleArm.setPower(0);
            WobbleGrab.setPosition(0);
            sleep(250);
            Drive(5, .5);
            Strafe(7, .75);
            // ^^ drives off wobble
            Drive(52, 1);
            // ^^ drives down to 2nd wobble
            rotate(165,.6);
            // ^^ rotates around
            Strafe(12, -.25);
            // ^^ lines up with the wobble HEHE
            Drive(6, -.25);
            sleep(100);
            WobbleGrab.setPosition(.38);
            sleep(200);
            Drive(67, .75);
            rotate(124, .6);
            WobbleGrab.setPosition(0);
            Drive(40, 1);

        } else if (pipeline.getAnalysis() > pipeline.ONE_RING_THRESHOLD) {
            webcam.stopStreaming();
            telemetry.addData("square", "middle ");
            telemetry.addData("Rings", "One");
            telemetry.update();

            ShooterArm.setPosition(0.259);
            ShootAngle.setPosition(0.785);
            WobbleGrab.setPosition(0.38);
            // Set for run
            Drive(80, -.5);
            // ^^ drives down to dropzone
            Strafe(13, -.5);
            // ^^ strafes into dropzone
            WobbleArm.setPower(-.75);
            sleep(1500);
            WobbleArm.setPower(0);
            WobbleGrab.setPosition(0);
            Shooter.setPower(1);
            sleep(500);
            // ^^ Drops wobble
            Drive(25, .5);
            // ^^ drives to shooting area
            WobbleArm.setPower(.75);
            ShootAngle.setPosition(.79);
            ShooterArm.setPosition(.259);
            sleep(500);
            WobbleArm.setPower(0);
            // ^^ sets up stuff to shoot 3 rings
            ShooterArm.setPosition(0);
            sleep(300);
            // ^^ shoots 1
            ShooterArm.setPosition(.259);
            sleep(300);
            // ^^ reloads 1
            ShooterArm.setPosition(0);
            sleep(300);
            // ^^ shoots 2
            ShooterArm.setPosition(.259);
            sleep(300);
            // ^^ reloads 2
            ShooterArm.setPosition(0);
            sleep(300);
            Intake.setPower(1);
            Shooter.setPower(0);
            ShootAngle.setPosition(1);
            // ^^ setups stuff for intake stack
            sleep(500);
            Drive(10, .25);
            Shooter.setPower(1);
            sleep(220);
            ShooterArm.setPosition(0);
            sleep(1000);
            ShootAngle.setPosition(.79);
            ShooterArm.setPosition(.259);
            sleep(850);
            ShooterArm.setPosition(0);
            Intake.setPower(0);
            WobbleArm.setPower(-.75);
            sleep(250);
            rotate(165, .6);
            Shooter.setPower(0);
            WobbleArm.setPower(0);
            //rotate for other wobble
            Strafe(17, .5);
            // ^^ strafes to line up with wobble
            Drive(13, -.25);
            sleep(250);
            WobbleGrab.setPosition(.38);
            sleep(250);
            // ^^ stuff
            Drive(46, .5);
            // ^^ drives to dropzone
            rotate(-150, .5);
            // ^^ rotates wobble into dropzone
            WobbleGrab.setPosition(0);
            sleep(300);
            Drive(10, .5);

        } else {

            webcam.stopStreaming();
            telemetry.addData("square ", "close");
            telemetry.addData("Rings", "None");
            telemetry.update();

            ShooterArm.setPosition(0.259);
            ShootAngle.setPosition(0.79);
            WobbleGrab.setPosition(0.38);
            sleep(100);
            Drive(64, -.5);
            Strafe(20, -.5);
            sleep(100);
            Shooter.setPower(1);
            sleep(100);
            ShootAngle.setPosition(.79);
            sleep(100);
            Drive(4, .5);
            sleep(100);
            ShooterArm.setPosition(.259);
            sleep(400);
            ShooterArm.setPosition(0);
            sleep(400);
            ShooterArm.setPosition(.259);
            sleep(400);
            ShooterArm.setPosition(0);
            sleep(400);
            ShooterArm.setPosition(.259);
            sleep(400);
            ShooterArm.setPosition(0);
            sleep(400);
            Strafe(24, -.5);
            sleep(100);
            Drive(2, .5);
            sleep(100);
            Drive(3, -.5);
            Shooter.setPower(0);
            sleep(500);
            WobbleArm.setPower(-.75);
            sleep(1500);
            WobbleArm.setPower(0);
            WobbleGrab.setPosition(0);
            sleep(100);
            Drive(10, .25);
            Strafe(19, .25);
            sleep(100);
            rotate(180, .3);
            Drive(20, -.25);
            sleep(200);
            WobbleGrab.setPosition(.38);
            sleep(200);
            Drive(28, .5);
            sleep(100);
            rotate(100, .6);
            WobbleGrab.setPosition(0);
            sleep(200);
            Drive(5, .5);
            sleep(100);
            rotate(40, .6);
            sleep(100);
            Drive(13, -.5);
            WobbleArm.setPower(.75);
            sleep(400);
            WobbleArm.setPower(0);

        }

    }

    public void Shoot(double Shots, double Power, double Angle) {
        for (double i = 0; i < Shots; i++) {
            Shooter.setPower(Power);
            ShootAngle.setPosition(Angle);
            sleep(275);
            ShooterArm.setPosition(0.005);
            sleep(275);
            ShooterArm.setPosition(.259);
        }
    }

    public void Drive(int Inch, double Power) {
        int DistanceTicks = ConvertInchesToRotations(Inch);
        ResetEncoders();

        while (Math.abs(RightRear.getCurrentPosition()) < DistanceTicks
                && opModeIsActive()) {

            telemetry.addData("Target"   , DistanceTicks);
            telemetry.addData("EncoderLF",LeftFront .getCurrentPosition());
            telemetry.addData("EncoderLR",LeftRear  .getCurrentPosition());
            telemetry.addData("EncoderRR",RightRear .getCurrentPosition());
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            correction = checkDirection();

            LeftFront .setPower(Power - correction);
            RightFront.setPower(Power + correction);
            LeftRear  .setPower(Power - correction);
            RightRear .setPower(Power + correction);

        }
        LeftFront .setPower(0);
        RightFront.setPower(0);
        LeftRear  .setPower(0);
        RightRear .setPower(0);

    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {

        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }


        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        //location95
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(290,110);
        //Size
        static final int REGION_WIDTH  = 25;
        static final int REGION_HEIGHT = 30;
        //Threshholds
        final int FOUR_RING_THRESHOLD = 155;
        final int ONE_RING_THRESHOLD  = 138;
        //idk stuff down from here
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;

            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }

    }

    public void Strafe  (double Inch, double Power) {
        int DistanceTicks = ConvertInchesToRotations(Inch);

        ResetEncoders();


        while (Math.abs(LeftFront.getCurrentPosition() )< DistanceTicks
                && opModeIsActive()) {

            correction = 0;
            LeftFront.setPower(+Power - correction);
            LeftRear.setPower(-Power + correction);
            RightFront.setPower(-Power - correction);
            RightRear.setPower(+Power + correction);
            telemetry.addData("Target", DistanceTicks);
            telemetry.addData("EncoderLF", LeftFront.getCurrentPosition());
            telemetry.update();
        }
        LeftFront .setPower(0);
        RightFront.setPower(0);
        LeftRear  .setPower(0);
        RightRear .setPower(0);


    }

    public int ConvertInchesToRotations(double Inch) {
        double Rotations = Inch / 11.87;
        return (int) (Rotations * ENCODER_TICKS_PER_ROTATION);
    }

    public void ResetEncoders() {
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .05;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private void rotate(int degrees, double power) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;

        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        LeftFront .setPower(leftPower);
        RightFront.setPower(rightPower);
        LeftRear  .setPower(leftPower);
        RightRear .setPower(rightPower);
        telemetry.addData("angle", getAngle());
        telemetry.update();
        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        LeftFront .setPower(0);
        RightFront.setPower(0);
        LeftRear  .setPower(0);
        RightRear .setPower(0);
        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

}
