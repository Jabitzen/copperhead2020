package org.firstinspires.ftc.teamcode.Introductory;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="BootSnakeRightAuto", group="Pushbot")
//@Disabled
public class BootSnakeRightAuto extends LinearOpMode {




    //HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware fixme init prob

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);                    // FIXME: 8/1/2019  -> parantheses?
        CustomOpMode opMode = new CustomOpMode(this);


        waitForStart();

        opMode.leftDrive.setPower(-0.5);
        opMode.rightDrive.setPower(-0.5);

        sleep(1500);

        opMode.leftDrive.setPower(0);
        opMode.rightDrive.setPower(0);

        sleep(1000);

        opMode.dropJewel.setPosition(0.3);
        sleep(1000);









    }
}

