package org.firstinspires.ftc.teamcode.pretty.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pretty.hardware;

@TeleOp
public class WristSubsystem extends LinearOpMode {

    //====================
    //scoring constants

    final double WRISTDOWN = 60; //degrees
    final double WRISTUP = 240; //degrees

    final double WXD = 1/240;

    //scoring constants
    //====================

    private hardware robot = new hardware();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //====================
        //scoring variables

        boolean xPressed = false;
        boolean yPressed = false;

        boolean pivotUp = false;
        boolean pivotDown = false;

        int wristXOffest = 0;
        int wristYOffset = 0;

        double wristXTarget = WRISTUP;
        double wristYTarget = robot.wristY.getPosition();

        //scoring variables
        //====================


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

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

            if (pivotDown) {
                wristXTarget = WRISTDOWN;
            } else if (pivotUp) {
                wristXTarget = WRISTUP;
            } else {
                wristXTarget = WRISTDOWN;
            }

            robot.wristX.setPosition( (wristXTarget + wristXOffest) * WXD);

            //scoring functions
            //====================


            //====================
            //telemetry

            telemetry.addData("pivotUp", pivotUp);
            telemetry.addData("pivotDown", pivotDown);

            telemetry.addData("WristXPos", robot.wristX.getPosition());
            telemetry.addData("WristXTarget", wristXTarget);

            telemetry.update();

            //telemetry
            //====================


            //====================
            //debug functions

            if (gamepad1.a) {
                terminateOpModeNow();
            }

            //debug functions
            //====================

        }
    }
}
