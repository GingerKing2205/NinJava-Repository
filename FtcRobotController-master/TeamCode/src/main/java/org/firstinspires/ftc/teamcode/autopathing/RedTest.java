package org.firstinspires.ftc.teamcode.autopathing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autopathing.RedRight;

@Autonomous
public class RedTest extends LinearOpMode {
    RedRight Right = new RedRight();

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (1==1) {
            Right.runOpMode(hardwareMap, telemetry);
        }
    }
}
