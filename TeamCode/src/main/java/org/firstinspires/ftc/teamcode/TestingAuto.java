package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

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

@Disabled
    @Autonomous
    public class TestingAuto extends LinearOpMode
    {

        DcMotor leftMotor, rightMotor;
        TouchSensor touch;
        BNO055IMU imu;
        Orientation lastAngles = new Orientation();
        double                  globalAngle, power = .30, correction;
        OpenCvCamera webcam;
        org.firstinspires.ftc.teamcode.vision.ShootAuto.SkystoneDeterminationPipeline pipeline;
        public DcMotor LeftFront  = null;
        public DcMotor LeftRear   = null;
        public DcMotor RightFront = null;
        public DcMotor RightRear  = null;
        double WHEEL_CIRCUMFERENCE = 3.78;
        double ENCODER_TICKS_PER_ROTATION = 537.6;



        @Override
        public void runOpMode()
        {


//------------------------------PhoneHardWareMap--------------------------------------------------\\

            LeftRear   = hardwareMap.dcMotor.get(" BackLeft   ");
            LeftFront  = hardwareMap.dcMotor.get(" FrontLeft  ");
            RightFront = hardwareMap.dcMotor.get(" FrontRight ");
            RightRear  = hardwareMap.dcMotor.get(" BackRight  ");

//------------------------------Direction---------------------------------------------------------\\
            //Reverse spins motors to the right Forward spins motors to the left
            LeftFront .setDirection (DcMotorSimple.Direction.FORWARD);
            LeftRear  .setDirection (DcMotorSimple.Direction.FORWARD);
            RightFront.setDirection (DcMotorSimple.Direction.REVERSE);
            RightRear .setDirection (DcMotorSimple.Direction.REVERSE);

//------------------------------Encoder---------------------------------------------------------\\

            LeftFront .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftRear  .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RightRear .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            LeftFront  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LeftRear   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RightFront .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RightRear  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            waitForStart();
            Strafe(1000,.05);











                }








        public void Drive   (int Inch, double Power) {
            int DistanceTicks = ConvertInchesToRotations(Inch);
            ResetEncoders();



            while (Math.abs(LeftFront.getCurrentPosition() )< DistanceTicks
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

        public void Strafe  (double Inch, double Power) {
            int DistanceTicks = ConvertInchesToRotations(Inch);

            ResetEncoders();


            while (Math.abs(LeftFront.getCurrentPosition() )< DistanceTicks
                    && opModeIsActive())
            {
                LeftFront .setPower(-Power);
                LeftRear  .setPower(Power);
                RightFront.setPower(Power);
                RightRear .setPower(-Power);
                telemetry.addData("Target"   , DistanceTicks);
                telemetry.addData("EncoderLF",LeftFront .getCurrentPosition());
                telemetry.addData("EncoderLR",LeftRear  .getCurrentPosition());

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

        /**
         * See if we are moving in a straight line and if not return a power correction value.
         * @return Power adjustment, + is adjust left - is adjust right.
         */
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


    }



