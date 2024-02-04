package org.firstinspires.ftc.teamcode.Disabled;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
