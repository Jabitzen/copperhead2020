package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Vision.BitMapVision;

@Autonomous(name="redQuarryAuto", group="12596")
public class redQuarryAuto extends LinearOpMode {

    BitMapVision bm1 = null;



    @Override
    public void runOpMode() throws InterruptedException {
        bm1 = new BitMapVision(this);
        waitForStart();

        String pos = bm1.findSkystones();
        sleep(3000);
        telemetry.addData("Skystone Position: ", pos);
        telemetry.update();
        sleep(5000);
    }
}
