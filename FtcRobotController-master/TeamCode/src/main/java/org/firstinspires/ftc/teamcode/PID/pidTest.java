package org.firstinspires.ftc.teamcode.PID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp

public class pidTest extends OpMode{
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f=0;

    public static int target = 0;

    private final double ticks_in_degree = 5264.0 / 180;

    private DcMotorEx leftFlip;
    @Override
    public void init() {

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFlip = hardwareMap.get(DcMotorEx.class, "leftFlip");


    }//ends init

    @Override
    public void loop() {

        controller.setPID(p, i, d);
        int armPos = leftFlip.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = - Math.cos(Math.toRadians(target / ticks_in_degree)) * f;


        double power = pid + ff;

        leftFlip.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.addData("power", power);
        telemetry.addData("ff", ff);
        telemetry.addData("pid", pid);
        telemetry.update();

    }//ends loop

}
