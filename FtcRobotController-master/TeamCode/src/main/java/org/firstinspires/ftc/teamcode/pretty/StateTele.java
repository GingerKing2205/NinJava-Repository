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
public class StateTele extends LinearOpMode {

    private hardware robot = new hardware();


    //====================
    //scoring constants

    final double WRISTDOWN = 60; //degrees
    final double WRISTUP = 240; //degrees
    final double PIVOTFLAT = 0; //degrees
    final double PIVOTUP = 120; //degrees

    final double PXD = 29.84; //Ticks To Degrees  TTI = Ticks To Inches
    final double PYD = 11.32;
    final double WXD = 1/240;
    final double SI = 2462 / 17;// ticks / inches

    final double SL = 15; //inches
    final double PHEIGHT = 6.5; //inches

    //scoring constants
    //====================


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //====================
        //scoring variables

        boolean xPressed = false;
        boolean yPressed = false;
        boolean lBump = false;
        boolean rBump = false;

        boolean aPressed = false;
        boolean bPressed = false;

        boolean leftClosed = true;
        boolean rightClosed = true;
        boolean pivotUp = false;
        boolean pivotDown = false;



        int wristXOffest = 0;
        int wristYOffset = 0;
        int pivotXOffset = 0;
        int pivotYOffset = 0;

        double wristXTarget = WRISTUP;
        double wristYTarget = robot.wristY.getPosition();
        double pivotXTarget = 0;
        double pivotYTarget = 0;
        double slideTarget = 0;

        //scoring variables
        //====================


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double pivotXDegrees = robot.pivotX.getCurrentPosition() / PXD;
            double pivotYDegrees = robot.pivotY.getCurrentPosition() / PYD;
            double slideInches = robot.slide.getCurrentPosition() / SI;


            //====================
            //scoring functions

            if (gamepad2.y && !yPressed) {
                if (pivotDown) {
                    pivotDown = false;
                } else if (!pivotDown) {
                    pivotUp = true;
                }
                yPressed = true;
            } else {
                yPressed = false;
            }

            if (gamepad2.x && !xPressed) {
                if (pivotUp) {
                    pivotUp = false;
                } else if (!pivotUp) {
                    pivotDown = true;
                }
                xPressed = true;
            } else {
                xPressed = false;
            }

            /*
            if (gamepad2.right_trigger > 0) {
                //if (slideTarget < 2462) {
                    slideTarget -= gamepad2.right_trigger;
                //}
            } else if (gamepad2.left_trigger > 0) {
                //if (slideTarget > 0) {
                    slideTarget += gamepad2.left_trigger;
                //}
            }
            */


            if (gamepad2.a && aPressed) {
                slideTarget -= 1/8;
                aPressed = true;
            } else {
                aPressed = false;
            }

            if (gamepad2.b && bPressed) {
                slideTarget += 1/8;
                bPressed = true;
            } else {
                bPressed = false;
            }

            if (pivotDown) {
                pivotXTarget = -Math.asin(PHEIGHT / (slideInches + SL));
                wristXTarget = WRISTDOWN - pivotXTarget;
            } else if (pivotUp) {
                pivotXTarget = PIVOTUP;
                wristXTarget = WRISTUP;
            } else {
                pivotXTarget = PIVOTFLAT;
                wristXTarget = WRISTDOWN;
            }

            if (gamepad2.right_bumper) {

                if (!rightClosed && !rBump) {
                    robot.clawR.setPosition(270);
                    rightClosed = true;
                } else if (!rBump){
                    robot.clawR.setPosition(0);
                    rightClosed = false;
                }
                rBump = true;
            } else {
                rBump = false;
            }

            if (gamepad2.left_bumper) {
                if (!leftClosed && !lBump) {
                    robot.clawL.setPosition(270);
                    leftClosed = true;
                } else if (!lBump){
                    robot.clawL.setPosition(0);
                    leftClosed = false;
                }
                lBump = true;
            } else {
                lBump = false;
            }

            robot.slide.setTargetPosition((int) (slideTarget * SI));
            robot.pivotX.setTargetPosition((int) (pivotXTarget * PXD));
            robot.wristX.setPosition( (wristXTarget + wristXOffest) * WXD);


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


            //====================
            //debug functions

            if (gamepad1.a) {
                terminateOpModeNow();
            }

            //debug functions
            //====================


            //====================
            //telemetry

            telemetry.addData("pivotUp", pivotUp);
            telemetry.addData("pivotDown", pivotDown);

            telemetry.addData("pivotXDegrees", pivotXDegrees);
            telemetry.addData("pivotXTarget", pivotXTarget);

            telemetry.addData("pivotYDegrees", pivotYDegrees);
            telemetry.addData("pivotYTarget", pivotYTarget);

            telemetry.addData("slideInches", slideInches);
            telemetry.addData("slideTarget", slideTarget);

            telemetry.addData("WristXPos", robot.wristX.getPosition());
            telemetry.addData("WristXTarget", wristXTarget);

            telemetry.update();

            //telemetry
            //====================

        }
    }
}
