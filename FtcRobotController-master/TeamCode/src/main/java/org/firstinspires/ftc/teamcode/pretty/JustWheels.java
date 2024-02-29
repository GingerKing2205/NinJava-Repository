package org.firstinspires.ftc.teamcode.pretty;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


import java.util.List;

import org.firstinspires.ftc.teamcode.pretty.hardware;

@TeleOp
public class JustWheels extends LinearOpMode {

    private hardware robot = new hardware();


    //====================
    //scoring constants


    //scoring constants
    //====================


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        //robot.runMode();

        //====================/
        //drivetrain variables


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

        boolean rbump = false;
        boolean lbump = false;

        double pivotXDegrees = robot.pivotX.getCurrentPosition() / 29.84;
        double pivotYDegrees = robot.pivotY.getCurrentPosition() / 11.32;
        double slideInches = robot.slide.getCurrentPosition();
        robot.pivotX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivotX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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


        //drivetrain functions
        //====================


        //====================
        //scoring functions
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            int currentx = robot.pivotX.getCurrentPosition();
            telemetry.addData("pivotx steps", currentx);
            telemetry.addData("WristX", robot.wristX.getPosition());
            telemetry.addData("Slide", robot.slide.getCurrentPosition());
            telemetry.update();
            if (gamepad2.dpad_up) {
                // robot.pivotX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // robot.pivotX.setTargetPosition(currentx - 50);
                robot.pivotX.setPower(1);
            } else if (gamepad2.dpad_down) {
                //robot.pivotX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // robot.pivotX.setTargetPosition(currentx + 50);
                robot.pivotX.setPower(-1);
            } else {
                robot.pivotX.setPower(0);
            }

            if (gamepad2.dpad_left) {
                robot.pivotY.setPower(-1);
            } else if (gamepad2.dpad_right) {
                robot.pivotY.setPower(1);
            } else {
                robot.pivotY.setPower(0);
            }
            if (gamepad2.left_trigger != 0) {
                robot.slide.setPower(1);
            } else if (gamepad2.right_trigger != 0) {
                robot.slide.setPower(-1);
            } else {
                robot.slide.setPower(0);
            }


            if (gamepad2.right_bumper) {

                if (!rightClosed && !rbump) {
                    robot.clawR.setPosition(270);
                    rightClosed = true;
                } else if (!rbump){
                    robot.clawR.setPosition(0);
                    rightClosed = false;
                }
                rbump = true;
            } else {
                rbump = false;
            }

            if (gamepad2.left_bumper) {
                if (!leftClosed && !lbump) {
                    robot.clawL.setPosition(270);
                    leftClosed = true;
                } else if (!lbump){
                    robot.clawL.setPosition(0);
                    leftClosed = false;
                }
                lbump = true;
            } else {
                lbump = false;
            }

            if (gamepad2.x) {
                robot.wristY.setPosition(1);

            }

            if(gamepad2.b) {
                robot.wristY.setPosition(0);
            }

            if (gamepad2.a) {
                terminateOpModeNow();
            }



            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x; //i made this neg to try and fix turningn

            double frontLeftPowerRaw = (y + x + (rx * 1));
            double frontRightPowerRaw = (y - x - (rx * 1));
            double backLeftPowerRaw = (y - x + (rx * 1));
            double backRightPowerRaw = (y + x - (rx * 1));
            if (!gamepad1.a) {//normal speed
                robot.frontL.setPower(frontLeftPowerRaw * -1);
                robot.backL.setPower(backLeftPowerRaw * 1);
                robot.frontR.setPower(frontRightPowerRaw * 1);
                robot.backR.setPower(backRightPowerRaw * -1);
            } else {
                robot.frontL.setPower(frontLeftPowerRaw * -.3);
                robot.backL.setPower(backLeftPowerRaw * -.3);
                robot.frontR.setPower(frontRightPowerRaw * -.3);
                robot.backR.setPower(backRightPowerRaw * -.3);
            }


            robot.frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        }
    }
}