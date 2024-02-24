package org.firstinspires.ftc.teamcode.pretty;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.rr.util.Encoder;

import java.util.List;

import org.firstinspires.ftc.teamcode.pretty.hardware;

@TeleOp
public class EncoderReset extends LinearOpMode{

    private hardware robot = new hardware();


    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.encoderReset();
        sleep(500);

        terminateOpModeNow();

    }
}
