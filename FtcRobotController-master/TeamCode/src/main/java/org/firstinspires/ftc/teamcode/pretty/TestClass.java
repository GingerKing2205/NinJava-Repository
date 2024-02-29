package org.firstinspires.ftc.teamcode.pretty;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pretty.hardware;

@TeleOp
public class TestClass extends LinearOpMode {


    private hardware robot = new hardware();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.clawL.setPosition(270);
        robot.clawR.setPosition(0);
    }

}
