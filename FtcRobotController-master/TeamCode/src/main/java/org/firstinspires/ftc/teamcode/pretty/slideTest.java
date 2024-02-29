package org.firstinspires.ftc.teamcode.pretty;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pretty.hardware;

@TeleOp
public class slideTest extends LinearOpMode {

    //====================
    //scoring constants

    final double PIVOTFLAT = 0; //degrees
    final double PIVOTUP = 110; //degrees

    final double WRISTDOWN = 0; //degrees
    final double WRISTUP = 180; //degrees
    final double WRISTCENTER = 25; //degrees


    final double SI = 2462 / 17;// ticks / inches

    final double PXD = 29.84; //Ticks To Degrees  TTI = Ticks To Inches
    final double PYD = PXD;// is 29.84

    final double WXD = 150;
    final double WYD = 50;


    final double SIDELEFT = -25;
    final double SIDERIGHT = 25;


    final double SL = 15; //inches
    final double PHEIGHT = 4; //inches was 3.5

    final double slideExtend = -18;

    final double slideRetract = 0;

    //scoring constants
    //====================

    private hardware robot = new hardware();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        robot.slidereset();
        robot.runMode();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //====================
        //scoring variables

        boolean xPressed = false;
        boolean yPressed = false;

        boolean upPressed = false;
        boolean downPressed = false;
        boolean leftPressed = false;
        boolean rightPressed = false;
        boolean startPressed = false;
        boolean backPressed = false;

        boolean up1Pressed = false;
        boolean down1Pressed = false;
        boolean left1Pressed = false;
        boolean right1Pressed = false;

        boolean pivotUp = false;
        boolean pivotDown = false;

        double slideTarget = 0;

        int pivotXOffset = 0;
        int pivotYOffset = 0;

        double pivotXTarget = 0;
        double pivotYTarget = 0;

        int wristXOffset = 0;
        int wristYOffset = 0;

        double wristXTarget = WRISTUP;
        double wristYTarget = WRISTCENTER;



        //scoring variables
        //====================


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double pivotXDegrees = robot.pivotX.getCurrentPosition() / PXD;
            double pivotYDegrees = robot.pivotY.getCurrentPosition() / PYD;
            double slideInches = robot.slide.getCurrentPosition() / SI;


            if (gamepad2.right_trigger != 0) {
                if (robot.slide.getCurrentPosition() > -3500) {
                    robot.slide.setPower(1);
                } else{
                    robot.slide.setPower(0);
                }
            } else if (gamepad2.left_trigger != 0) {
                if (robot.slide.getCurrentPosition() <= 0) {
                    robot.slide.setPower(-1);
                } else {
                    robot.slide.setPower(0);
                }
            } else {
                robot.slide.setPower(0);
            }

            telemetry.addData("slideSteps", robot.slide.getCurrentPosition());
            telemetry.addData("slideInches", slideInches);
            telemetry.update();

        }
    }
}
