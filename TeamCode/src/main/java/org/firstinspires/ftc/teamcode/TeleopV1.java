package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="TeleopV1", group="Pushbot")
//@Disabled

public class TeleopV1 extends OpMode{

    public DcMotor leftDrive;
    public DcMotor rightDrive;



    //HardwareMap hwMap = null;

  //  HardwarePushbot robot       = new HardwarePushbot();

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */


        leftDrive  = hardwareMap.dcMotor.get("left_drive");
                //hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       // robot.block.setPosition(0);

    }

    /*@Override
    public void init_loop() {
    }*/                         //do i need yay or nay

    public void loop() {
        double forward;
        double backward;
        double left;
        double right;
        double arcLeftStick;
        double arcRightStick;
        //boolean intake;
       // boolean block;


        forward = -gamepad1.left_stick_y;
        if (Math.abs(-gamepad1.left_stick_y) > 0.05) {
            leftDrive.setPower(forward);
            rightDrive.setPower(forward);
        }

        else {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }


        left = gamepad1.right_stick_x;
        if (Math.abs(gamepad1.right_stick_x) > 0.05) {
            leftDrive.setPower(-left);
            rightDrive.setPower(left);
        }

        else {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }

        /*intake = gamepad2.left_bumper;
        if (gamepad2.left_bumper) {
            robot.intake.setPower(1);
        }                                     fixme

        else {
            robot.intake.setPower(0);
        } */

       /* block = gamepad2.right_bumper;
        if (gamepad2.right_bumper) {           fixme
            robot.block.setPosition(1);
        } */





    }
}
