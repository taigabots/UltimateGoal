package org.firstinspires.ftc.teamcode.vision;


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
public class WobbleAuto extends LinearOpMode
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

        LeftRear    = hardwareMap.dcMotor.get (" BackLeft   ");
        LeftFront   = hardwareMap.dcMotor.get (" FrontLeft  ");
        RightFront  = hardwareMap.dcMotor.get (" FrontRight ");
        RightRear   = hardwareMap.dcMotor.get (" BackRight  ");
        Intake      = hardwareMap.dcMotor.get ( "Intake      ");
        Shooter     = hardwareMap.dcMotor.get ( "Shooter     ");
        ShooterArm  = hardwareMap.servo.get   ( "ShooterArm  ");
        ShootAngle  = hardwareMap.servo.get   ( "ShooterAngle");
        WobbleArm   = hardwareMap.dcMotor.get ( "WobbleArm    ");
        WobbleGrab  = hardwareMap.servo.get   ( "WobbleGrab    ");

//------------------------------Direction---------------------------------------------------------\\
        //Reverse spins motors to the right Forward spins motors to the left
        LeftFront .setDirection (DcMotorSimple.Direction.FORWARD);
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
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
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






        if      (pipeline.getAnalysis()>pipeline.FOUR_RING_THRESHOLD)
        {

            webcam.stopStreaming();
            telemetry.addData("square","Far");
            telemetry.addData("Ring", "FOUR");
            telemetry.update();

            ShooterArm.setPosition(.259);
            WobbleGrab.setPosition(.45);
            Drive (15,-.75);
            // ^^ Drive off wall
            Strafe(24,-.75);
            // ^^ Strafes to line up with wobble sqaure
            Drive (80,-.75);
            // ^^ Drives to far dropzone
            WobbleArm.setPower(-.5);
            sleep(2000);
            WobbleArm.setPower(0);
            WobbleGrab.setPosition(0);
//-------------------^ PLACES WOBBLE^-------------------------------------------------------------\\

            Drive (5,.5);
            Strafe(28,.5);
            // ^^ Strafe to shoot for high
            Drive (37, .75);
            // ^^ Drives past parkline
            ShootAngle.setPosition(.80);
            Shooter.setPower(1);
            sleep(1000);
            // ^^ revs up motor and flips moble back
            ShooterArm.setPosition(0.005);
            sleep(500);
            ShooterArm.setPosition(.259);
            sleep(500);
            ShooterArm.setPosition(0.005);
            sleep(500);
            ShooterArm.setPosition(.259);
            sleep(500);
            ShooterArm.setPosition(0.005);
            sleep(500);
   //---------------^shoots starting 3^-----------------------------------------------------------\\
            ShooterArm.setPosition(.259);
            ShootAngle.setPosition(1);
            Shooter.setPower(0);
            Intake.setPower(1);
            // ^^ sets the stuff up for the 4 intake rings
            Strafe(2,-.75);
            // ^^ strafe to line intake up with rings
            sleep(250);
            Drive(20,.5);
            // ^^ Intake the 4 rings
            sleep(1250);
            Intake.setPower(0);
            ShootAngle.setPosition(.8);
            Shooter.setPower(1);
            sleep(1000);
            ShooterArm.setPosition(0.005);
            // ^^ aims shooter and stop intake
            sleep(500);
            ShooterArm.setPosition(.259);
            sleep(500);
            ShooterArm.setPosition(0.005);
            //^^ shoot 1 reset
            sleep(500);
            ShooterArm.setPosition(.259);
            sleep(500);
            ShooterArm.setPosition(0.005);
            //^^ shoot  2 reset
            sleep(500);
            ShooterArm.setPosition(.259);
            ShootAngle.setPosition(1);
            Shooter.setPower(0);
            Drive(27,-.75);



        }
        else if (pipeline.getAnalysis()>pipeline.ONE_RING_THRESHOLD)
        {
            webcam.stopStreaming();
            telemetry.addData("square","middle ");
            telemetry.addData("Rings", "One");
            telemetry.update();

        }
        else
        {

            webcam.stopStreaming();
            telemetry.addData("square ","close");
            telemetry.addData( "Rings","None");
            telemetry.update();

        }

    }

    public void Drive   (int Inch, double Power) {
        int DistanceTicks = ConvertInchesToRotations(Inch);
        ResetEncoders();




        while (Math.abs(RightRear.getCurrentPosition() )< DistanceTicks
                && opModeIsActive())
        {

            telemetry.addData("Target"   , DistanceTicks);
            telemetry.addData("EncoderLF",LeftFront .getCurrentPosition());
            telemetry.addData("EncoderLR",LeftRear  .getCurrentPosition());
            telemetry.addData("EncoderRR",RightRear .getCurrentPosition());
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            double LFSped = 1;
            double LRSped = 1;
            double RFSped = 1;
            double RRSped = 1;

            correction = checkDirection();
            if (Power > 0)
            {
                LFSped = .8;
                LRSped = .8;
                RFSped = 1;
                RRSped = 1;
            }
            else
            {
                LFSped = -.89;
                LRSped = -.89;
                RFSped = -1;
                RRSped = -1;
            }




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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(95,95);
        //Size
        static final int REGION_WIDTH  = 50;
        static final int REGION_HEIGHT = 30;
        //Threshholds
        final int FOUR_RING_THRESHOLD = 155;
        final int ONE_RING_THRESHOLD  = 135;
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
                && opModeIsActive())
        {
            LeftFront .setPower(Power);
            LeftRear  .setPower(-Power);
            RightFront.setPower(-Power);
            RightRear .setPower(Power);
            telemetry.addData("Target"   , DistanceTicks);
            telemetry.addData("EncoderLF",LeftFront .getCurrentPosition());
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
        LeftFront.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRear.setMode  (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRear.setMode  (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode (DcMotor.RunMode.RUN_USING_ENCODER);


    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {
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

    private double checkDirection()
    {
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

    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
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
