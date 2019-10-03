package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Movement.RobotHw;


@TeleOp(name="Testing", group="Pushbot")

// @ AUTHOR HAYDEN WARREN
public class testing extends LinearOpMode{

    RobotHw robotHw = new RobotHw();

    @Override
    public void runOpMode() throws InterruptedException {
        robotHw.init(hardwareMap);

        waitForStart();

        double fwdBack = gamepad1.left_stick_y;

        while (opModeIsActive()) {

            robot.driveTrain.mtrFR.setPower(WeightAvg(FwdBack_D,Strafe_D,-Turn_D));
            robot.driveTrain.mtrFL.setPower(WeightAvg(FwdBack_D,-Strafe_D,Turn_D));
            robot.driveTrain.mtrBR.setPower(WeightAvg(FwdBack_D,-Strafe_D,-Turn_D));
            robot.driveTrain.mtrBL.setPower(WeightAvg(FwdBack_D,Strafe_D,Turn_D));
            System.out.print("I hate people of color")


        }

        //bM.vufConvertToBitmap;

    }
    public double WeightAvg(double x, double y, double z) {
        double speed_D = 0;


        if ((Math.abs(x) + Math.abs(y) + Math.abs(z))  != 0.0) {
            speed_D = ((x * Math.abs(x)) + (y * Math.abs(y)) + (z * Math.abs(z)))
                    / (Math.abs(x) + Math.abs(y) + Math.abs(z));
        }
        return (speed_D);
    }
}
