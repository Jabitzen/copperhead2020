package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@Autonomous(name="BootSnakeLeftAuto", group="Pushbot")
//@Disabled
public class BootSnakeLeftAuto extends LinearOpMode {
    
    CustomOpMode myOpMode;


    //HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware fixme init prob

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);                    // FIXME: 8/1/2019  -> parantheses?
        myOpMode = new CustomOpMode(this);


        waitForStart();

        //opMode.dropJewel.setPosition(0.8);
        myOpMode.jewel();
        
        myOpMode.moveStraight(0.5, 1);

        //sleep(1000);



        //turn_left(0.5, 100);

        //move_straight(0.5, 100);

       // opMode.dropJewel.setPosition(0.3);

        //move


        //opMode.moveStraight(0.5, 100);

        /*opMode.dropJewel.setPosition(1);

        opMode.moveStraight(-0.5, 300);

        opMode.turn(-1, 0.5, 500);

        opMode.moveStraight(0.5, 150); */










    }

















}

