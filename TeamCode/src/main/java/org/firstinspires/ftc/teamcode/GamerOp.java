package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class GamerOp extends OpMode {


    public DcMotor LeftFront  = null;
    public DcMotor LeftRear   = null;
    public DcMotor RightFront = null;
    public DcMotor RightRear  = null;
    public DcMotor Intake     = null;
    public DcMotor Shooter    = null;
    public Servo   ShootAngle = null;
    public Servo   ShooterArm = null;

//------------------------------InitLoop----------------------------------------------------------\\

    @Override
    public void init() {


//------------------------------PhoneHardWareMap--------------------------------------------------\\

        LeftRear    = hardwareMap.dcMotor.get (" BackLeft    ");
        LeftFront   = hardwareMap.dcMotor.get (" FrontLeft   ");
        RightFront  = hardwareMap.dcMotor.get (" FrontRight  ");
        RightRear   = hardwareMap.dcMotor.get (" BackRight   ");
        Intake      = hardwareMap.dcMotor.get ( "Intake      ");
        Shooter     = hardwareMap.dcMotor.get ( "Shooter     ");
        ShooterArm  = hardwareMap.servo.get   ( "ShooterArm  ");
        ShootAngle  = hardwareMap.servo.get   ( "ShooterAngle");

//------------------------------Direction---------------------------------------------------------\\

        //Reverse spins motors to the right Forward spins motors to the left
        LeftFront .setDirection (DcMotorSimple.Direction.FORWARD);
        LeftRear  .setDirection (DcMotorSimple.Direction.FORWARD);
        RightFront.setDirection (DcMotorSimple.Direction.REVERSE);
        RightRear .setDirection (DcMotorSimple.Direction.REVERSE);
        Intake    .setDirection (DcMotorSimple.Direction.REVERSE);
        Shooter   .setDirection (DcMotorSimple.Direction.REVERSE);

        LeftFront .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRear  .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

//------------------------------OpMode------------------------------------------------------------\\

    @Override
    public void loop() {

//------------------------------DriverController--------------------------------------------------\\

        double Drive  = +gamepad1.left_stick_y ;
        double Strafe = +gamepad1.left_stick_x ;
        double Turn   = +gamepad1.right_stick_x;
        double Speed  = 1;
        double SpeedTurn = .75;





        if (gamepad1.right_bumper)
        {
            Speed = .5;
            SpeedTurn = .5;
        }
        else
        {
            Speed = 1;
            SpeedTurn = .75;
        }

        LeftFront  .setPower( - Drive + Strafe * Speed + Turn * SpeedTurn);
        LeftRear   .setPower( - Drive - Strafe * Speed + Turn * SpeedTurn);
        RightFront .setPower( - Drive - Strafe * Speed - Turn * SpeedTurn);
        RightRear  .setPower( - Drive + Strafe * Speed - Turn * SpeedTurn);

        telemetry.addData("Lf",LeftFront  .getCurrentPosition());
        telemetry.addData("LR",LeftRear   .getCurrentPosition());
        telemetry.addData("RF",RightFront .getCurrentPosition());
        telemetry.addData("RR",RightRear  .getCurrentPosition());
        telemetry.addData("Y ",gamepad1.left_stick_y);
        telemetry.addData("X ",gamepad1.left_stick_x);
        telemetry.update();

//------------------------------Intake/Belt-------------------------------------------------------\\


        if(gamepad1.right_trigger > .1)
        {
            Intake.setPower(1);
        }
        else if (gamepad1.left_trigger > .1)
        {
            Intake.setPower(-1);
        }
        else
        {
            Intake.setPower(0);
        }


        if(gamepad2.right_trigger > .1)
        {
            Intake.setPower(1);
        }
        else if (gamepad2.left_trigger > .1)
        {
            Intake.setPower(-1);
        }
        else
        {
            Intake.setPower(0);
        }



//------------------------------Shooter-----------------------------------------------------------\\

        if (gamepad2.left_bumper)
        {
            Shooter.setPower(1);
        }
        else
        {
            Shooter.setPower(0);
        }

        if (gamepad2.right_bumper)
        {
            ShooterArm.setPosition(0.005);
        }
        else
        {
            ShooterArm.setPosition(.259);
        }

        if (gamepad2.dpad_up)
        {
            ShootAngle.setPosition(0.7);
        }
        else if (gamepad2.dpad_down)
        {
            ShootAngle.setPosition(1);
        }else if (gamepad2.dpad_right)
        {
            ShootAngle.setPosition(.725);
        }else if (gamepad2.dpad_left)
        {
            ShootAngle.setPosition(.75);
        }


//------------------------------Wobble------------------------------------------------------------\\

        if(gamepad2.dpad_up)
        {

        }
        else if (gamepad2.dpad_down)
        {

        }
        else if(gamepad2.dpad_right)
        {

        }

    }
}
