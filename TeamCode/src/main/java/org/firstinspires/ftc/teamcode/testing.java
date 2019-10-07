package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.Movement.RobotHw;


@TeleOp(name="Testing", group="Pushbot")

// @ AUTHOR HAYDEN WARREN
public class testing extends LinearOpMode{


    public DcMotor fL = null;
    public DcMotor fR = null;
    public DcMotor bL = null;
    public DcMotor bR = null;

    @Override
    public void runOpMode() throws InterruptedException {
        fL  = hardwareMap.get(DcMotor.class, "fL");
        fR  = hardwareMap.get(DcMotor.class, "fR");
        bL  = hardwareMap.get(DcMotor.class, "bL");
        bR  = hardwareMap.get(DcMotor.class, "bR");

        waitForStart();
        while (opModeIsActive()) {
            //mecanumDrive_Cartesian(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            trigMecanum();
        }




    }
    public void mecanumDrive_Cartesian(double x, double y, double rotation)
    {
        double wheelSpeeds[] = new double[4];

        wheelSpeeds[0] = x + y + rotation;
        wheelSpeeds[1] = -x + y - rotation;
        wheelSpeeds[2] = -x + y + rotation;
        wheelSpeeds[3] = x + y - rotation;

        normalize(wheelSpeeds);

        fL.setPower(-wheelSpeeds[0]);
        fR.setPower(wheelSpeeds[1]);
        bL.setPower(wheelSpeeds[2]);
        bR.setPower(wheelSpeeds[3]);
    }   //mecanumDrive_Cartesian

    public void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);

        for (int i = 1; i < wheelSpeeds.length; i++)
        {
            double magnitude = Math.abs(wheelSpeeds[i]);

            if (magnitude > maxMagnitude)
            {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0)
        {
            for (int i = 0; i < wheelSpeeds.length; i++)
            {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }

    public void trigMecanum() {
        telemetry.addLine("start");
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        telemetry.addData("fl", v1);
        telemetry.addData ("fR", v2);
        telemetry.addData ("bL", v3);
        telemetry.addData ("bR", v4);
        telemetry.addData ("Right X", rightX);
        telemetry.update();

        fL.setPower(v1);
        fR.setPower(v2);
        bL.setPower(v3);
        bR.setPower(v4);
    }


}
