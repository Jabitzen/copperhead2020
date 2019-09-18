package org.firstinspires.ftc.teamcode.Introductory;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="BootSnakeTeleop", group="Pushbot")
@Disabled

public class BootSnakeTeleop extends OpMode{

    HardwarePushbot robot       = new HardwarePushbot();

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //robot.block.setPosition(0);

    }

    @Override
    public void init_loop() {
    }

    public void loop() {
        double forward;
        double backward;
        double left;
        double right;
        boolean intake;
        boolean block;


        forward = -gamepad1.left_stick_y;
        if (-gamepad1.left_stick_y > 0) {
            robot.leftDrive.setPower(0.5);
            robot.rightDrive.setPower(0.5);
        }

        else {
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
        }


        backward = gamepad1.left_stick_y;
        if (gamepad1.left_stick_y < 0) {
            robot.leftDrive.setPower(-0.5);
            robot.rightDrive.setPower(-0.5);
        }

        else {
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
        }

        left = gamepad1.right_stick_x;
        if (gamepad1.right_stick_x < 0) {
            robot.leftDrive.setPower(-0.5);
            robot.rightDrive.setPower(0.5);
        }

        right = -gamepad1.right_stick_x;
        if (-gamepad1.right_stick_x > 0) {
            robot.leftDrive.setPower(0.5);
            robot.rightDrive.setPower(-0.5);
        }

        else {
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
        }

        intake = gamepad2.left_bumper;
        if (gamepad2.left_bumper) {
            //robot.intake.setPower(1);
        }

        else {
            //robot.intake.setPower(0);
        }

        block = gamepad2.right_bumper;
        if (gamepad2.right_bumper) {
            //robot.block.setPosition(1);
        }

    }
}
