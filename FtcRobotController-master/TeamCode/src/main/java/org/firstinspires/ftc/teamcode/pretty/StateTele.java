package org.firstinspires.ftc.teamcode.pretty;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.pretty.hardware;
import org.firstinspires.ftc.teamcode.pretty.subsystems.*;

@TeleOp
@Config
public class StateTele extends LinearOpMode {


    public hardware robot = new hardware();
    public WristSubsystem wrist = new WristSubsystem();

    @Override
    public void runOpMode() throws InterruptedException {
        
        robot.init(hardwareMap, true);
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
        boolean aPressed = false;

        double slideTarget = 0;

        boolean rbump = false;
        boolean lbump = false;
        boolean leftClosed = true;
        boolean rightClosed = true;

        robot.clawR.setPosition(270);
        robot.clawL.setPosition(0);


        //scoring variables
        //====================


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            //====================
            //scoring variables

            Globals.readPositions(robot.pivotX.getCurrentPosition(), robot.pivotY.getCurrentPosition(), robot.slide.getCurrentPosition());

            //scoring variables
            //====================


            //====================
            //scoring functions

            if (gamepad2.right_bumper) {

                if (!rightClosed && !rbump) {
                    robot.clawR.setPosition(Globals.CLAWRCLOSED);
                    rightClosed = true;
                } else if (!rbump){
                    robot.clawR.setPosition(Globals.CLAWROPEN);
                    rightClosed = false;
                }
                rbump = true;
            } else {
                rbump = false;
            }

            if (gamepad2.left_bumper) {
                if (!leftClosed && !lbump) {
                    robot.clawL.setPosition(Globals.CLAWLCLOSED);
                    leftClosed = true;
                } else if (!lbump){
                    robot.clawL.setPosition(Globals.CLAWLOPEN);
                    leftClosed = false;
                }
                lbump = true;
            } else {
                lbump = false;
            }



            if (gamepad2.right_trigger != 0) {
                if (robot.slide.getCurrentPosition() > Globals.SLIDEXTEND) {
                    robot.slide.setPower(-1);
                    PivotSubsystem.set();
                    wrist.set();
                } else{
                    robot.slide.setPower(0);
                }
            } else if (gamepad2.left_trigger != 0) {
                if (robot.slide.getCurrentPosition() <= Globals.SLIDERETRACT) {
                    robot.slide.setPower(1);
                    PivotSubsystem.set();
                    wrist.set();
                } else {
                    robot.slide.setPower(0);
                }
            } else {
                robot.slide.setPower(0);
            }

            if (gamepad2.a) {
                if (!Globals.wristStored && !aPressed) {
                    Globals.pivotstored();
                } else if (!aPressed) {
                    Globals.pivotNeutral();
                }
                PivotSubsystem.set();
                wrist.set();
                aPressed = true;
            } else {
                aPressed = false;
            }

            if (gamepad2.start) {
                Globals.pivotLeft = false;
                Globals.pivotRight = true;
                PivotSubsystem.set();
                wrist.set();
            }

            if (gamepad2.back) {
                Globals.pivotLeft = true;
                Globals.pivotRight = false;
                PivotSubsystem.set();
                wrist.set();
            }

            if (gamepad2.b) {
                Globals.stackUp();

                PivotSubsystem.set();
                wrist.set();
            }

            if (gamepad2.y) {
                Globals.pivotScoring();
                PivotSubsystem.set();
                wrist.set();
            }

            if (gamepad2.x) {
                if (!xPressed) {
                    if (Globals.pivotUp || Globals.pivotDown) {
                        Globals.pivotNeutral();
                    } else if (!Globals.pivotUp) {
                        Globals.pivotIntaking();
                    }
                }
                PivotSubsystem.set();
                wrist.set();
                xPressed = true;
            } else {
                xPressed = false;
            }

            robot.pivotX.setTargetPosition((int) PivotSubsystem.xCalculate());
            robot.pivotY.setTargetPosition((int) PivotSubsystem.yCalculate());

            robot.wristX.setPosition(wrist.xCalculate());
            robot.wristY.setPosition(wrist.yCalculate());


            if (gamepad2.dpad_up) {
                if (!upPressed) {
                    PivotSubsystem.moveOffset(true, false);
                    PivotSubsystem.set();
                    wrist.set();
                }
                upPressed = true;
            } else {
                upPressed = false;
            }

            if (gamepad2.dpad_down) {
                if (!downPressed) {
                    PivotSubsystem.moveOffset(true, true);
                    PivotSubsystem.set();
                    wrist.set();
                }
                downPressed = true;
            } else {
                downPressed = false;
            }

            if (gamepad2.dpad_left) {
                if (!leftPressed) {
                    PivotSubsystem.moveOffset(false, true);
                    PivotSubsystem.set();
                    wrist.set();
                }
                leftPressed = true;
            } else {
                leftPressed = false;
            }

            if (gamepad2.dpad_right) {
                if (!rightPressed) {
                    PivotSubsystem.moveOffset(false, false);
                    PivotSubsystem.set();
                    wrist.set();
                }
                rightPressed = true;
            } else {
                rightPressed = false;
            }

            if (gamepad2.guide || !Globals.pivotUp) {
                PivotSubsystem.resetOffset();
                PivotSubsystem.set();
                wrist.set();
            }

            //scoring functions
            //====================


            //====================
            //telemetry

            telemetry.addData("pivotUp", Globals.pivotUp);
            telemetry.addData("pivotDown", Globals.pivotDown);
            telemetry.addData("pivotStored", Globals.wristStored);

            telemetry.addData("pivotXDegrees", Globals.pivotXDegrees);
            telemetry.addData("pivotYDegrees", Globals.pivotYDegrees);

            telemetry.addData("pivotXTarget", Globals.pivotXTarget);
            telemetry.addData("pivotYTarget", Globals.pivotYTarget);

            telemetry.addData("WristXPos", robot.wristX.getPosition() * Globals.WXD);

            telemetry.addData("WristYPos", robot.wristY.getPosition() * Globals.WYD);

            telemetry.addData("slideSteps", robot.slide.getCurrentPosition());
            telemetry.addData("slideInches", Globals.slideInches);

            telemetry.addData("wristStored", Globals.wristStored);
            telemetry.addData("aPressed" , aPressed);

            telemetry.addData("stackHeight", Globals.stackHeight);

            telemetry.update();

            //telemetry
            //====================


            //====================
            //drivetrain variables

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            //double throttle = gamepad1.right_trigger;

            double frontLeftPowerRaw = (y + x + (rx * -.7));
            double backLeftPowerRaw = (y - x + (rx * -.7));
            double frontRightPowerRaw = (y - x - (rx * -.7));
            double backRightPowerRaw = (y + x - (rx * -.7));

            double orientation = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            //drivetrain variables
            //====================


            //====================
            //drivetrain functions

            if ((frontLeftPowerRaw == 0) && (frontRightPowerRaw == 0) && (backRightPowerRaw == 0) && (backLeftPowerRaw == 0)) {
                robot.frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            } else {
                robot.frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            }

            if (!gamepad1.b) {//normal speed
                robot.frontL.setPower(-frontLeftPowerRaw);
                robot.backL.setPower(-backLeftPowerRaw);
                robot.frontR.setPower(-frontRightPowerRaw);
                robot.backR.setPower(-backRightPowerRaw);
            }

            if (gamepad1.b) {
                if (orientation > -88 && orientation <= 90) {
                    robot.frontL.setPower(frontLeftPowerRaw * .7);
                    robot.backL.setPower(backLeftPowerRaw * .7);
                    robot.frontR.setPower(frontRightPowerRaw * -.7);
                    robot.backR.setPower(backRightPowerRaw * -.7);
                } else if (orientation > 90 && orientation < 181 || orientation > -181 && orientation < -92 ) {
                    robot.frontL.setPower(frontLeftPowerRaw * .7);
                    robot.backL.setPower(backLeftPowerRaw * .7);
                    robot.frontR.setPower(frontRightPowerRaw * -.7);
                    robot.backR.setPower(backRightPowerRaw * -.7);
                }
            }

            //drivetrain functions
            //====================


            //====================
            //endgame functions

            if (gamepad1.start) {
                robot.plane.setPower(1);
            } else {
                robot.plane.setPower(0);
            }

            if (gamepad1.back) {
                robot.pivotX2.setPower(-1);

                Globals.pivotIntaking();
                PivotSubsystem.set();
            } else {
                robot.pivotX2.setPower(0);
            }

            //endgame functions
            //====================


            //====================
            //debug functions

            if (gamepad1.a) {
                terminateOpModeNow();
            }


            if (gamepad1.dpad_up) {
                if (!up1Pressed) {
                    wrist.moveOffset(true, false);
                    wrist.set();
                }
                up1Pressed = true;
            } else {
                up1Pressed = false;
            }

            if (gamepad1.dpad_down) {
                if (!down1Pressed) {
                    wrist.moveOffset(true, true);
                    wrist.set();
                }
                down1Pressed = true;
            } else {
                down1Pressed = false;
            }

            if (gamepad1.dpad_left) {
                if (!left1Pressed) {
                    wrist.moveOffset(false, true);
                    wrist.set();
                }
                left1Pressed = true;
            } else {
                left1Pressed = false;
            }

            if (gamepad1.dpad_right) {
                if (!right1Pressed) {
                    wrist.moveOffset(false, false);
                    wrist.set();
                }
                right1Pressed = true;
            } else {
                right1Pressed = false;
            }

            if (gamepad1.guide) {
                wrist.resetOffset();
                wrist.set();
            }

            //debug functions
            //====================


        }
    }
}
