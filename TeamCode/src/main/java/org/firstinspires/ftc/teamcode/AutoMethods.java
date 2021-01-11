package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.ShootAuto;
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
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@Autonomous
public class AutoMethods extends LinearOpMode {




    public DcMotor LeftFront  = null;
    public DcMotor LeftRear   = null;
    public DcMotor RightFront = null;
    public DcMotor RightRear  = null;
    double WHEEL_CIRCUMFERENCE = 3.78;
    double ENCODER_TICKS_PER_ROTATION = 537.6;
    double FOUR = 160;
    double ONE  = 100;
    double NONE = 0  ;
    public OpenCvWebcam  webcam;
    ShootAuto.SkystoneDeterminationPipeline pipeline;



    @Override
    public void runOpMode() throws InterruptedException {



        telemetry.addData("status","Initialized");
        telemetry.update();

//------------------------------WebcamSetup-------------------------------------------------------\\

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

//------------------------------PhoneHardWareMap--------------------------------------------------\\

        LeftRear   = hardwareMap.dcMotor.get(" BackLeft   ");
        LeftFront  = hardwareMap.dcMotor.get(" FrontLeft  ");
        RightFront = hardwareMap.dcMotor.get(" FrontRight ");
        RightRear  = hardwareMap.dcMotor.get(" BackRight  ");

//------------------------------Direction---------------------------------------------------------\\
        //Reverse spins motors to the right Forward spins motors to the left
        LeftFront.setDirection (DcMotorSimple.Direction.FORWARD);
        LeftRear.setDirection  (DcMotorSimple.Direction.FORWARD);
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightRear.setDirection (DcMotorSimple.Direction.REVERSE);

//------------------------------Encoder---------------------------------------------------------\\

        LeftFront .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRear  .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear .setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("EncoderLF",LeftFront .getCurrentPosition());
            telemetry.addData("EncoderLR",LeftRear  .getCurrentPosition());
            telemetry.addData("EncoderRR",RightRear .getCurrentPosition());
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }




    }

//------------------------------OpMode------------------------------------------------------------\\

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,150);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

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
        private volatile ShootAuto.SkystoneDeterminationPipeline.RingPosition position = ShootAuto.SkystoneDeterminationPipeline.RingPosition.FOUR;

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

            position = ShootAuto.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = ShootAuto.SkystoneDeterminationPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = ShootAuto.SkystoneDeterminationPipeline.RingPosition.ONE;
            }else{
                position = ShootAuto.SkystoneDeterminationPipeline.RingPosition.NONE;
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
    public void Drive   (int Inch, double Power) {
        int DistanceTicks = ConvertInchesToRotations(Inch);

        ResetEncoders();

            LeftFront .setPower(Power);
            RightFront.setPower(Power);
            LeftRear  .setPower(Power);
            RightRear .setPower(Power);
            while (Math.abs(LeftFront.getCurrentPosition() )< DistanceTicks
            && opModeIsActive())
            {
                telemetry.addData("Target"   , DistanceTicks);
                telemetry.addData("EncoderLF",LeftFront .getCurrentPosition());
                telemetry.addData("EncoderLR",LeftRear  .getCurrentPosition());
                telemetry.addData("EncoderRF",RightFront.getCurrentPosition());
                telemetry.addData("EncoderRR",RightRear .getCurrentPosition());
                telemetry.update();
            }
            LeftFront .setPower(0);
            RightFront.setPower(0);
            LeftRear  .setPower(0);
            RightRear .setPower(0);

        }

    public void GyroTurn(double angle, double Power) {
        double thing = (angle / angle) * -1;
        LeftFront .setPower(Power * +thing);
        RightFront.setPower(Power * +thing);
        LeftRear  .setPower(Power * -thing);
        RightRear .setPower(Power * -thing);
    }

    public void Strafe  (double Inch, double Power) {
        int DistanceTicks = ConvertInchesToRotations(Inch);

        ResetEncoders();

        LeftFront .setPower(Power);
        RightFront.setPower(-Power);
        LeftRear  .setPower(-Power);
        RightRear .setPower(Power);
        while (Math.abs(LeftFront.getCurrentPosition() )< DistanceTicks
                && opModeIsActive())
        {
            telemetry.addData("Target"   , DistanceTicks);
            telemetry.addData("EncoderLF",LeftFront .getCurrentPosition());
            telemetry.addData("EncoderLR",LeftRear  .getCurrentPosition());

            telemetry.addData("EncoderRR",RightRear .getCurrentPosition());
            telemetry.update();
        }
        Off();


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
        LeftRear.setMode  (DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode (DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void Off() {

        LeftFront.setPower(0);
        LeftRear.setPower(0);
        RightFront.setPower(0);
        RightRear.setPower(0);

    }



}
