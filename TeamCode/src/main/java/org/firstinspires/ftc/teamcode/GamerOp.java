package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class GamerOp extends OpMode {


    public DcMotor LeftFront  = null;
    public DcMotor LeftRear   = null;
    public DcMotor RightFront = null;
    public DcMotor RightRear  = null;
    //public DcMotor Intake     = null;
    //public DcMotor Shooter    = null;

//------------------------------InitLoop----------------------------------------------------------\\

    @Override
    public void init() {


//------------------------------PhoneHardWareMap--------------------------------------------------\\

        LeftRear    = hardwareMap.dcMotor.get (" BackLeft   ");
        LeftFront   = hardwareMap.dcMotor.get (" FrontLeft  ");
        RightFront  = hardwareMap.dcMotor.get (" FrontRight ");
        RightRear   = hardwareMap.dcMotor.get (" BackRight  ");
        //Intake      = hardwareMap.dcMotor.get ( "Intake     ");
        //Shooter     = hardwareMap.dcMotor.get ( "Shooter    ");

//------------------------------Direction---------------------------------------------------------\\

        //Reverse spins motors to the right Forward spins motors to the left
        LeftFront .setDirection (DcMotorSimple.Direction.REVERSE);
        LeftRear  .setDirection (DcMotorSimple.Direction.FORWARD);
        RightFront.setDirection (DcMotorSimple.Direction.FORWARD);
        RightRear .setDirection (DcMotorSimple.Direction.REVERSE);
        //Intake    .setDirection (DcMotorSimple.Direction.REVERSE);
        //Shooter   .setDirection (DcMotorSimple.Direction.REVERSE);


        LeftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

//------------------------------OpMode------------------------------------------------------------\\

    @Override
    public void loop() {

//------------------------------DriverController--------------------------------------------------\\

        double Drive  = -gamepad1.left_stick_y ;
        double Strafe = +gamepad1.left_stick_x ;
        double Turn   = +gamepad1.right_stick_x;

        LeftFront  .setPower( + Drive - Strafe - Turn);
        LeftRear   .setPower( + Drive + Strafe - Turn);
        RightFront .setPower( + Drive + Strafe + Turn);
        RightRear  .setPower( + Drive - Strafe + Turn);


//------------------------------Intake/Belt-------------------------------------------------------\\

        //Intake.setPower(+gamepad1.right_trigger);
        //Intake.setPower(-gamepad1.left_trigger );

//------------------------------Shooter-----------------------------------------------------------\\

    if (gamepad1.a)
    {
        //Shooter.setPower(1);
    }



    }

}
