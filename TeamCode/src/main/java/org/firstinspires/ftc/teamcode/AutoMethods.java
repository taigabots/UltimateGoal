package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvInternalCamera2;


@Autonomous
public class AutoMethods extends LinearOpMode {

    public DcMotor LeftFront = null;
    public DcMotor LeftRear = null;
    public DcMotor RightFront = null;
    public DcMotor RightRear = null;
    double WHEEL_CIRCUMFERENCE = 11.87;
    double ENCODER_TICKS_PER_ROTATION = 537.6;
    double COUNTS_PER_INCH = 6367;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("status","Initialized");
        telemetry.update();

//------------------------------PhoneHardWareMap--------------------------------------------------\\

        LeftRear   = hardwareMap.dcMotor.get(" BackLeft   ");
        LeftFront  = hardwareMap.dcMotor.get(" FrontLeft  ");
        RightFront = hardwareMap.dcMotor.get(" FrontRight ");
        RightRear  = hardwareMap.dcMotor.get(" BackRight  ");

//------------------------------Direction---------------------------------------------------------\\
        //Reverse spins motors to the right Forward spins motors to the left
        LeftFront.setDirection (DcMotorSimple.Direction.REVERSE);
        LeftRear.setDirection  (DcMotorSimple.Direction.REVERSE);
        RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        RightRear.setDirection (DcMotorSimple.Direction.FORWARD);

//------------------------------Encoder---------------------------------------------------------\\

        LeftFront .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRear  .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightRear .setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        encoderDrive(.25,5,5,4);
        telemetry.addData("EncoderLF",LeftFront .getCurrentPosition());
        telemetry.addData("EncoderLR",LeftRear  .getCurrentPosition());
        telemetry.addData("EncoderRF",RightFront.getCurrentPosition());
        telemetry.addData("EncoderRR",RightRear .getCurrentPosition());
        telemetry.update();

    }

//------------------------------OpMode------------------------------------------------------------\\



    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;


        if (opModeIsActive()) {


            newLeftTarget  = LeftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = RightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            LeftFront.setTargetPosition(newLeftTarget);
            RightFront.setTargetPosition(newRightTarget);

            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            LeftFront.setPower(Math.abs(speed));
            RightFront.setPower(Math.abs(speed));
            LeftRear.setPower(Math.abs(speed));
            RightRear.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LeftFront.isBusy() && RightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        LeftFront.getCurrentPosition(),
                        RightFront.getCurrentPosition());
                telemetry.update();
            }

           Off();

            // Turn off RUN_TO_POSITION
            LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }





    public void Strafe(double Inches, double Power) {


    }

    public void ConvertInchesToRotations(double Distance) {

    }

    public void ResetEncoders() {
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void Off() {

        LeftFront.setPower(0);
        LeftRear.setPower(0);
        RightFront.setPower(0);
        RightRear.setPower(0);

    }



}
