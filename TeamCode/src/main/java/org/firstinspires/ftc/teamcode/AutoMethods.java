package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.RingSense;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class AutoMethods extends LinearOpMode {




    public DcMotor LeftFront  = null;
    public DcMotor LeftRear   = null;
    public DcMotor RightFront = null;
    public DcMotor RightRear  = null;
    double WHEEL_CIRCUMFERENCE = 3.78;
    double ENCODER_TICKS_PER_ROTATION = 537.6;
    public OpenCvWebcam  webcam;
    RingSense.SkystoneDeterminationPipeline pipeline;


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
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
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
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        telemetry.addData("EncoderLF",LeftFront .getCurrentPosition());
        telemetry.addData("EncoderLR",LeftRear  .getCurrentPosition());
        //telemetry.addData("EncoderRF",RightFront.getCurrentPosition());
        telemetry.addData("EncoderRR",RightRear .getCurrentPosition());
        telemetry.update();

    }

//------------------------------OpMode------------------------------------------------------------\\

    public void Drive(int Inch, double Power)
        {
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
    public void GyroTurn(double angle, double Power)
    {

    }
    public void Strafe(double Inch, double Power) {
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
