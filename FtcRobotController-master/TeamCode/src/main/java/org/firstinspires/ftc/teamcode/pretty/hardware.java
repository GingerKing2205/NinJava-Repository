package org.firstinspires.ftc.teamcode.pretty;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

import javax.annotation.concurrent.GuardedBy;

@Disabled
@Config
public class hardware {

    //drivetrain

    public DcMotorEx frontL,frontR, backL, backR;

    public List<DcMotorEx> motors;

    //odo encoders

    //arm
    public DcMotorEx slide;
    public DcMotorEx pivotX, pivotY;
    public Servo wristX, wristY;
    public Servo clawL, clawR;

    //endgame
    public CRServo plane;

    //hardwareMap
    public HardwareMap hardwareMap;

    //imu
    public IMU imu;


    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        this.frontL = hardwareMap.get(DcMotorEx.class, "frontL");
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontL.setDirection(DcMotorSimple.Direction.REVERSE);

        this.frontR = hardwareMap.get(DcMotorEx.class, "frontR");
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.backL = hardwareMap.get(DcMotorEx.class, "backL");
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backL.setDirection(DcMotorSimple.Direction.REVERSE);


        this.backR = hardwareMap.get(DcMotorEx.class, "backR");
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);



        this.slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        this.pivotX = hardwareMap.get(DcMotorEx.class, "pivotX");


        this.pivotY = hardwareMap.get(DcMotorEx.class, "pivotY");


        this.wristX = hardwareMap.get(Servo.class, "wristX");

        this.wristY = hardwareMap.get(Servo.class, "wristY");

        this.clawL = hardwareMap.get(Servo.class, "clawL");

        this.clawR = hardwareMap.get(Servo.class, "clawR");


        this.plane = hardwareMap.get(CRServo.class, "plane");
    }

    public void encoderReset() {

        this.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.pivotX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.pivotY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void slidereset() {
        this.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runMode() {
       // this.slide.setTargetPosition(slide.getCurrentPosition());
       // this.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       // this.slide.setVelocity(8000);
        this.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.pivotX.setTargetPosition(pivotX.getCurrentPosition());
        this.pivotX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.pivotX.setVelocity(6000);

        this.pivotY.setTargetPosition(pivotY.getCurrentPosition());
        this.pivotY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.pivotY.setVelocity(4000);
    }


}



