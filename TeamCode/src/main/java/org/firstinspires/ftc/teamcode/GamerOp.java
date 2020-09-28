package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class GamerOp extends OpMode {

//------------------------------InitLoop----------------------------------------------------------\\

    @Override
    public void init() {

    }

//------------------------------OpMode------------------------------------------------------------\\

    @Override
    public void loop() {

//------------------------------DriverController--------------------------------------------------\\

        double Drive  = gamepad1.left_stick_y ;
        double Strafe = gamepad1.left_stick_x ;
        double Turn   = gamepad1.right_stick_x;
/*
        robot.LeftFront  .setPower( Drive - Strafe + Turn);
        robot.LeftRear   .setPower( Drive + Strafe + Turn);
        robot.RightFront .setPower( Drive - Strafe - Turn);
        robot.RightRear  .setPower( Drive + Strafe - Turn);
*/
//------------------------------Intake/Belt-------------------------------------------------------\\

    }

}
