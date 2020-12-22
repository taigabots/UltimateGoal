package org.firstinspires.ftc.teamcode;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Servo;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp
public class GamerOp extends OpMode {


//------------------------------GyroSetup?--------------------------------------------------------\\

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

//------------------------------InitSetup?--------------------------------------------------------\\

    public DcMotor LeftFront  = null;
    public DcMotor LeftRear   = null;
    public DcMotor RightFront = null;
    public DcMotor RightRear  = null;
    public DcMotor Intake     = null;
    public DcMotor Shooter    = null;
    public Servo   ShootAngle = null;
    public Servo   ShooterArm = null;
    public DcMotor WobbleArm  = null;

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
        WobbleArm .setDirection (DcMotorSimple.Direction.REVERSE);


        LeftFront .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRear  .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WobbleArm .setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        LeftFront  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LeftRear   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightFront .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RightRear  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//----------------------------------MoreGyro------------------------------------------------------\\

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

    }

//------------------------------OpMode------------------------------------------------------------\\

    @Override
    public void loop() {

//------------------------------DriverController--------------------------------------------------\\


        double D   = +gamepad1.left_stick_y ;
        double S   = +gamepad1.left_stick_x ;
        double T   = +gamepad1.right_stick_x;
        double Sped  = 1;
        double SpedT = .75;
        double GyroSquare = 0;

        if (gamepad1.right_bumper)
        {

            if (getAngle()> 4)
            {

                GyroSquare = .375;

            }
            else if (getAngle()< -4)
            {

                GyroSquare = -.375;

            }
            else
            {

                GyroSquare = 0;

            }

        }

        if(gamepad1.left_bumper)
        {
            Sped =  .75;
            SpedT = .4;
        }else
        {
            Sped = 1;
            SpedT = .75;
        }

        if (gamepad1.y)
        {
            resetAngle();
        }



        LeftFront  .setPower( - D + S * Sped + T * SpedT + GyroSquare);
        LeftRear   .setPower( - D - S * Sped + T * SpedT + GyroSquare);
        RightFront .setPower( - D - S * Sped - T * SpedT - GyroSquare);
        RightRear  .setPower( - D + S * Sped - T * SpedT - GyroSquare);
        telemetry.addData("angle",getAngle());
        telemetry.addData("Wobble",WobbleArm.getCurrentPosition());
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


        if      (gamepad2.right_trigger > .1)
        {
            Intake.setPower(1);
        }
        else if (gamepad2.left_trigger  > .1)
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

            ShootAngle.setPosition(0.735);

        }
        else if (gamepad2.dpad_down)
        {

            ShootAngle.setPosition(1);

        }
        else if (gamepad2.dpad_right)
        {

            ShootAngle.setPosition(.75);

        }
        else if (gamepad2.dpad_left)
        {

            ShootAngle.setPosition(.75);

        }


//------------------------------Wobble------------------------------------------------------------\\

        if(gamepad2.a)
        {
            double Target = 0;
            double Angle  = WobbleArm.getCurrentPosition();
            if(Target < Angle) {
                WobbleArm.setPower(.25);
            }
            else
                {
                    WobbleArm.setPower(0);
                }
        }
        else if(gamepad2.b)
        {
            double Target = 0;
            double Angle = WobbleArm.getCurrentPosition();
            if (Target > Angle)
            {
                WobbleArm.setPower(-.25);
            }
            else
            {
                WobbleArm.setPower(0);
            }
        }



    }



    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
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

}
