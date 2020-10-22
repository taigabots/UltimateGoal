package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class AutoMethods extends LinearOpMode {

    public DcMotor LeftFront = null;
    public DcMotor LeftRear = null;
    public DcMotor RightFront = null;
    public DcMotor RightRear = null;
    double WHEEL_CIRCUMFERENCE = 3.78;
    double ENCODER_TICKS_PER_ROTATION = 537.6;
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

        Strafe(22,-.5);
        Drive(55,-.5);
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
            // telemetry.addData("EncoderRF",RightFront.getCurrentPosition());
            telemetry.addData("EncoderRR",RightRear .getCurrentPosition());
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
