package org.firstinspires.ftc.teamcode.pretty;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
public class hardware {

    //drivetrain
    public DcMotorEx left_back_drive;
    public DcMotorEx left_front_drive;
    public DcMotorEx right_back_drive;
    public DcMotorEx right_front_drive;

    //odo encoders
    public Encoder parallelEncoder, perpendicularEncoder;

    //arm
    public DcMotorEx Slide;
    public DcMotorEx Noose;
    public DcMotorEx intake;
    public DcMotorEx leftFlip;

    //endgame
    public CRServo plane;
    public CRServo hang;

    //imu
    public BNO055IMU imu;

    private HardwareMap hardwareMap;

}
