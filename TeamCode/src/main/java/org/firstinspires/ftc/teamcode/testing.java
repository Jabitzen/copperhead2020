package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.Vision.BitMap;

@TeleOp(name="Testing", group="Pushbot")


public class testing extends LinearOpMode{

    BitMap bM = new BitMap(this);

    @Override
    public void runOpMode() throws InterruptedException {
        //bM.vufConvertToBitmap;
        bM.getBitmap();
        bM.sample();
    }
}
