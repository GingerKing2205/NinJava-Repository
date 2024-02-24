package org.firstinspires.ftc.teamcode.pretty.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pretty.hardware;

@TeleOp
public class PivotSubsystem extends LinearOpMode {


    private hardware robot = new hardware();


    //====================
    //scoring constants

    final double PIVOTFLAT = 0; //degrees
    final double PIVOTUP = 120; //degrees

    final double PXD = 29.84; //Ticks To Degrees  TTI = Ticks To Inches
    final double PYD = 11.32;

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

        boolean aPressed = false;
        boolean bPressed = false;

        boolean pivotUp = false;
        boolean pivotDown = false;

        int pivotXOffset = 0;
        int pivotYOffset = 0;

        double pivotXTarget = 0;
        double pivotYTarget = 0;

        //scoring variables
        //====================


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            //====================
            //scoring variables

            double pivotXDegrees = robot.pivotX.getCurrentPosition() / PXD;
            double pivotYDegrees = robot.pivotY.getCurrentPosition() / PYD;

            //scoring variables
            //====================


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
                pivotXTarget = -Math.asin(PHEIGHT / SL);
            } else if (pivotUp) {
                pivotXTarget = PIVOTUP;
            } else {
                pivotXTarget = PIVOTFLAT;
            }

            robot.pivotX.setTargetPosition((int) ( (pivotXTarget + pivotXOffset) * PXD));

            //scoring functions
            //====================


            //====================
            //telemetry

            telemetry.addData("pivotUp", pivotUp);
            telemetry.addData("pivotDown", pivotDown);

            telemetry.addData("pivotXDegrees", pivotXDegrees);
            telemetry.addData("pivotXTarget", pivotXTarget);

            telemetry.update();

            //telemetry
            //====================


            //====================
            //debug functions

            if (gamepad1.a) {
                terminateOpModeNow();
            }


            if (gamepad2.dpad_up && !aPressed) {
                pivotXOffset += 10;
                aPressed = true;
            } else {
                aPressed = false;
            }

            if (gamepad2.dpad_down && !bPressed) {
                pivotXOffset -= 10;
                bPressed = true;
            } else {
                bPressed = false;
            }

            //debug functions
            //====================

        }
    }
}
