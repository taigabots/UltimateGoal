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

//------------------------------InitLoop----------------------------------------------------------\\

    @Override
    public void init() {


//------------------------------PhoneHardWareMap--------------------------------------------------\\

        LeftRear    = hardwareMap.dcMotor.get (" BackLeft   ");
        LeftFront   = hardwareMap.dcMotor.get (" FrontLeft  ");
        RightFront  = hardwareMap.dcMotor.get (" FrontRight ");
        RightRear   = hardwareMap.dcMotor.get (" BackRight  ");

//------------------------------Direction---------------------------------------------------------\\

        LeftFront .setDirection(DcMotorSimple.Direction.REVERSE);
        LeftRear  .setDirection(DcMotorSimple.Direction.REVERSE);
        RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        RightRear .setDirection(DcMotorSimple.Direction.FORWARD);

        LeftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

//------------------------------OpMode------------------------------------------------------------\\

    @Override
    public void loop() {

//------------------------------DriverController--------------------------------------------------\\

        double Drive  = gamepad1.left_stick_y ;
        double Strafe = gamepad1.left_stick_x ;
        double Turn   = gamepad1.right_stick_x;

        LeftFront  .setPower( + Drive - Strafe - Turn);
        LeftRear   .setPower( + Drive + Strafe - Turn);
        RightFront .setPower( + Drive + Strafe + Turn);
        RightRear  .setPower( + Drive - Strafe + Turn);

        //telemetry.addData("TicksLF",LeftFront.getCurrentPosition());
        //telemetry.addData("TicksLR",LeftRear.getCurrentPosition());
        //telemetry.addData("TicksRF",RightRear.getCurrentPosition());
        //telemetry.addData("TicksRR",RightRear.getCurrentPosition());

        //telemetry.update();
//------------------------------Intake/Belt-------------------------------------------------------\\



    }

}
