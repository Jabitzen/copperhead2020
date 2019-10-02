package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.BitMap;

@Autonomous(name="redQuarryAuto", group="12596")
public class redQuarryAuto extends LinearOpMode {
    BitMap bm = new BitMap(this);


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        String pos = bm.findSkystones();
        sleep(3000);
        telemetry.addData("Skystone Position: ", pos);
        telemetry.update();
        sleep(5000);
    }
}
