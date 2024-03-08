package org.firstinspires.ftc.teamcode.pretty;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pretty.hardware;

@TeleOp
public class ATelecheck extends LinearOpMode{

    private hardware robot = new hardware();

    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap, false);
        robot.runMode();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad2.right_bumper) {
                robot.slide.setTargetPosition(2700);
            } else if (gamepad2.left_bumper) {
                robot.slide.setTargetPosition(0);
            } else {
                robot.slide.setTargetPosition(robot.slide.getCurrentPosition());
            }
            telemetry.addData("slide steps" , robot.slide.getCurrentPosition());
            telemetry.update();


        }

    }
}
