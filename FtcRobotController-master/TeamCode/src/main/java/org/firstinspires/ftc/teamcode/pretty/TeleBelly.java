package org.firstinspires.ftc.teamcode.pretty;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


import java.util.List;

import org.firstinspires.ftc.teamcode.pretty.hardware;

@Disabled
@TeleOp
public class TeleBelly extends LinearOpMode {

    private hardware robot = new hardware();


    //====================
    //scoring constants

    final double WRISTDOWN = 180; //degrees
    final double WRISTUP = 0; //degrees
    final double PIVOTFLAT = 0; //degrees
    final double PIVOTUP = 120; //degrees

    final double PXD = 29.84; //TTD = Ticks To Degrees  TTI = Ticks To Inches
    final double PYD = 11.32;
    final double SI = 1;

    final double SL = 15; //inches
    final double PHEIGHT = 6.5; //inches

    //scoring constants
    //====================


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, true);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //====================
            //scoring variables

            boolean xPressed = false;
            boolean yPressed = false;

            boolean leftClosed = true;
            boolean rightClosed = true;
            boolean pivotUp = false;
            boolean pivotDown = false;

            double pivotXDegrees = robot.pivotX.getCurrentPosition() / PXD;
            double pivotYDegrees = robot.pivotY.getCurrentPosition() / PYD;
            double slideInches = robot.slide.getCurrentPosition() / SI;

            int wristXOffest = 0;
            int wristYOffset = 0;
            int pivotXOffset = 0;
            int pivotYOffset = 0;

            double wristXTarget = pivotXDegrees + wristXOffest;
            double wristYTarget = pivotYDegrees + wristYOffset;
            double pivotXTarget = (int) slideInches + pivotXOffset;
            double pivotYTarget = pivotYOffset;
            double slideTarget = 0;

            //scoring variables
            //====================


            if (gamepad2.y && yPressed) {
                if (pivotDown) {
                    pivotDown = false;
                } else {
                    pivotUp = true;
                }
                yPressed = true;
            } else {
                yPressed = false;
            }

            if (gamepad2.x && xPressed) {
                if (pivotUp) {
                    pivotUp = false;
                } else {
                    pivotDown = true;
                }
                xPressed = true;
            } else {
                xPressed = false;
            }

            if (gamepad2.right_trigger > 0) {
                if (slideTarget < 100) {
                    slideTarget +=gamepad2.right_trigger;
                }
            } else if (gamepad2.left_trigger > 0) {
                if (slideTarget > 0) {
                    slideTarget -= gamepad2.left_trigger;
                }
            }

            if (pivotDown) {
                pivotXTarget = -Math.asin( PHEIGHT / (slideInches + SL) );
                wristXTarget = WRISTDOWN - pivotXTarget;
            } else if (pivotUp) {
                pivotXTarget = PIVOTUP;
                wristXTarget = WRISTUP;
            } else {
                pivotXTarget = PIVOTFLAT;
                wristXTarget = WRISTDOWN;
            }


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



            telemetry.update();

            //telemetry
            //====================

        }
    }
}