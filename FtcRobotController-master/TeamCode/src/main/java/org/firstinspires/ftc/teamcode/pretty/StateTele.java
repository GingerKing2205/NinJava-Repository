package org.firstinspires.ftc.teamcode.pretty;

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
public class StateTele extends LinearOpMode {

    private hardware robot = new hardware();


    //====================
    //scoring constants

    static Double WRISTDOWN = 0.1;

    //scoring constants
    //====================


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        //====================
        //drivetrain variables

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x; //i made this neg to try and fix turning

        double frontLeftPowerRaw = (y + x + (rx * -.7));
        double frontRightPowerRaw = (y - x - (rx * -.7));
        double backLeftPowerRaw = (y - x + (rx * -.7));
        double backRightPowerRaw = (y + x - (rx * -.7));

        //drivetrain variables
        //====================


        //====================
        //scoring variables

        boolean wristPressed = false;
        boolean pivotPressed = false;

        boolean leftClosed = true;
        boolean rightClosed = true;
        boolean wristUp = false;
        boolean pivotUp = false;

        double pivotXDegrees = robot.pivotX.getCurrentPosition()/29.84;
        double pivotYDegrees = robot.pivotY.getCurrentPosition()/11.32;
        double slideInches = robot.slide.getCurrentPosition();

        int wristXOffest = 0;
        int wristYOffset = 0;
        int pivotXOffset = 0;
        int pivotYOffset = 0;

        double wristXTarget = pivotXDegrees + wristXOffest;
        double wristYTarget = pivotYDegrees + wristYOffset;
        int pivotXTarget = (int) slideInches + pivotXOffset;
        int pivotYTarget = pivotYOffset;



        //scoring variables
        //====================


        //====================
        //drivetrain functions
        robot.frontL.setPower(frontLeftPowerRaw * -.7);
        robot.frontR.setPower(frontRightPowerRaw * -.7);
        robot.backL.setPower(backLeftPowerRaw * -.7);
        robot.backR.setPower(backRightPowerRaw * -.7);

        if ((frontLeftPowerRaw == 0) && (frontRightPowerRaw == 0) && (backRightPowerRaw == 0) && (backLeftPowerRaw == 0)) {
            robot.frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            robot.frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        if (gamepad1.a) {

        }

        //drivetrain functions
        //====================



        //====================
        //scoring functions


        if (gamepad2.right_trigger > 0 && slideInches < 36) {
            robot.slide.setPower(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger > 0 && slideInches > 0) {
            robot.slide.setPower(gamepad2.left_trigger);
        } else {
            robot.slide.setPower(0);
        }
        
        if (gamepad2.y && pivotPressed) {
            pivotUp = !pivotUp;
            pivotPressed = true;
        } else {
            pivotPressed = false;
        }

        if (pivotXDegrees > 90) {

            wristXTarget = (int) pivotXDegrees;

        } else {

            if (gamepad2.a && !wristPressed) {
                wristUp = !wristUp;
                wristPressed = true;
            } else {
                wristPressed = false;
            }

            if (wristUp) {
                wristXTarget = (int) slideInches;
            } else {
                wristXTarget = (int) (slideInches + WRISTDOWN);
            }

        }
        robot.wristX.setPosition(wristXTarget + wristXOffest);

        //scoring functions
        //====================



        //====================
        //endgame functions

        if (gamepad1.start) {
            robot.plane.setPower(0.5);
        } else {
            robot.plane.setPower(0);
        }

        //endgame functions
        //====================
    }
}
