package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Welcome 2020",group = "12596")
public class newYearTest extends LinearOpMode {
    public DcMotor motorR = null;
    public DcMotor motorL = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Hardware mapmed motors
        motorR = hardwareMap.get(DcMotor.class, "motorR");
        motorL = hardwareMap.get(DcMotor.class, "motorL");
        //Direction of motors
        motorR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorL.setDirection(DcMotorSimple.Direction.REVERSE);
        // Wait till play button
        waitForStart();

        while (opModeIsActive()) {
            // Variables
            double leftPower;
            double rightPower;
            double drive = -gamepad1.left_stick_y; // Forward/Back
            double turn  =  gamepad1.right_stick_x; // Left/Right

            // Calculate value for the motor
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Apply powers to motor
            motorR.setPower(rightPower);
            motorL.setPower(leftPower);

            //Random
        }
    }
}
