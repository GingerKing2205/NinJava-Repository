package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



@TeleOp
public class GinjaKarpTelePID extends LinearOpMode {

    //====================
    //pulls PID

    private liftPID arm = new liftPID();

    //pulls PID
    //====================


    //====================
    //creates arm pos Enum

    enum Level {
        STANDARD,
        LOW,
        MEDIUM,
        HIGH,
    }

    //creates arm pos Enum
    //====================


    //====================
    //create motors/servos
    private CRServo hang;
    private DcMotor Slide;
    private DcMotor Noose;
    private DcMotor left_back_drive;
    private DcMotor left_front_drive;
    private DcMotor right_back_drive;
    private DcMotor right_front_drive;
    private DcMotor intake;
    private CRServo plane;

    //create motors/servos
    //====================


    @Override
    public void runOpMode() throws InterruptedException {

        //====================
        //initPID


        arm.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //initPID
        //====================


        //====================
        //sets default enum arm pos

        Level height = Level.STANDARD;

        //sets default enum arm pos
        //====================


        //====================
        //create IMU

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //create IMU
        //====================


        //====================
        //assign motors/servos

        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Noose = hardwareMap.get(DcMotor.class, "Noose");
        left_back_drive = hardwareMap.get(DcMotor.class, "left_back_drive");
        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        right_back_drive = hardwareMap.get(DcMotor.class, "right_back_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        plane = hardwareMap.get(CRServo.class, "plane");
        hang = hardwareMap.get(CRServo.class, "hang");

        //assign motors/servos
        //====================


        //====================
        //configure motors/servos

        //set encoders
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Noose.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.reset();

        //set target positions
        Slide.setTargetPosition(0);
        Noose.setTargetPosition(550);

        //set direction
        left_back_drive.setDirection(DcMotor.Direction.REVERSE);
        left_front_drive.setDirection(DcMotor.Direction.REVERSE);

        //set zero power behavior
        left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Noose.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set run mode
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Noose.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //configure motors/servos
        //====================


        //====================
        //runOpMode variables

        boolean fieldCentric = false; //fieldCentric mode

        int gallow = -3300;//lift to backdrop position
        boolean downReleased = true;
        boolean upReleased = true;

        int knot =0;//Noose intake position


        //runOpMode variables
        //====================


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //====================
            //armPID


            arm.loop();


            //armPID
            //====================


            //====================
            //while variables

            double FlipStep = arm.leftFlip.getCurrentPosition();
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x; //i made this neg to try and fix turning

            //while variables
            //====================


            //====================
            //telemetry

            telemetry.update();
           /* telemetry.addData("slide", Slide.getCurrentPosition());
           */ telemetry.addData("Noose", Noose.getCurrentPosition()); /*
            telemetry.addData("NooseTarget", Noose.getTargetPosition());
            telemetry.addData("NoosePower", Noose.getPower());
            */telemetry.addData("LeftFlip", FlipStep);/*
            telemetry.addData("fieldCentric", fieldCentric);
            telemetry.addData("liftTarget" , gallow);
            telemetry.addData("fieldCentric", fieldCentric); */
            telemetry.addData("pos" , arm.leftFlip.getCurrentPosition());
            telemetry.addData("target" , arm.target);
            telemetry.addData("pow", arm.leftFlip.getPower());
            telemetry.addData("pid", arm.controller.getPeriod());


            //telemetry
            //====================


            //====================
            //set velocities

            //((DcMotorEx) leftFlip).setVelocity(7000);
            ((DcMotorEx) Noose).setVelocity(4000);
            ((DcMotorEx) Slide).setVelocity(2000);

            //set velocities
            //====================


            //====================
            //functions
            if (gamepad1.start == true) {//hang release
                hang.setPower(.5);
            } else {
                hang.setPower(0);
            }

            if (gamepad2.start == true) {//plane launcher
                plane.setPower(1);
            } else {
                plane.setPower(0);
            }


            if (gamepad2.left_stick_y != 0) {//intake power
                intake.setPower(-gamepad2.left_stick_y);
            } else {
                intake.setPower(0);
            }

            if (gamepad2.right_trigger > 0) {//extends slide
                Slide.setTargetPosition(900);
            } else if (gamepad2.left_trigger > 0) {//retracts slide
                Slide.setTargetPosition(0);
            } else {//holds slides
                Slide.setTargetPosition(Slide.getCurrentPosition());
            }

            if (gamepad2.y == true) {//raises Flip
                arm.target = gallow;
                Noose.setTargetPosition((int) (FlipStep * 0.545 + knot + 550));//Intake-Ground Parallel
            }

            if (gamepad2.back == true) {//stack pull height
                Noose.setTargetPosition(knot + 300);
            }


            if (gamepad2.x == true) {//lowers flip
                Noose.setTargetPosition(knot + 550);
                arm.target = 0;


            }


            if (gamepad2.left_bumper == true) {//Intake-Flip parallel to
                if (FlipStep > -3500) {
                    Noose.setTargetPosition(knot + 550);
                } else {
                    Noose.setTargetPosition((int) (FlipStep * 0.545 + knot + 2000));//Intake-Ground Parallel
                }
            }


            if (gamepad2.right_bumper == true) {//lowers intake
                Noose.setTargetPosition(knot);
            }

            if (gamepad1.dpad_down == true) {//lowers intake position
                knot = knot - 25;
                Noose.setTargetPosition(knot);
            } else if (gamepad1.dpad_up == true) {//highers intake position
                knot = knot + 25;
                Noose.setTargetPosition(knot);
        }

            //functions
            //====================


            //====================
            //debugging

            /*if (gamepad2.dpad_right == true) {//lift encoder reset
                arm.reset();
            }*/

            if (gamepad2.dpad_up == true && upReleased) {//manual lift up
                upReleased = false;
                //gallow -= 50;
                //arm.target = gallow;

                if (height == Level.LOW) {
                    height = Level.MEDIUM;
                } else if (height == Level.MEDIUM){
                    height = Level.HIGH;
                } else {
                    height = Level.STANDARD;
                }

            } else if (gamepad2.dpad_up == false) {
                upReleased = true;
            }

            if (gamepad2.dpad_down == true && downReleased) {//manual lift down
                downReleased = false;
                //gallow += 50;
                //arm.target = gallow;

                if (height == Level.STANDARD) {
                    height = Level.LOW;
                } else {
                    height = Level.STANDARD;
                }

            } else if (gamepad2.dpad_down == false) {
                downReleased = true;
            }

            if (!downReleased || !upReleased) {
                switch (height) {
                    case STANDARD:
                        gallow = -3300;
                        break;
                    case LOW:
                        gallow = -4100;
                        break;
                    case MEDIUM:
                        gallow = -3900;
                        break;
                    case HIGH:
                        gallow = -3700;
                        break;
                }
                arm.target = gallow;
            }

            if (gamepad2.a == true) {//killswitch
                terminateOpModeNow();
                for (int count = 0; count < 10; count++) {
                }
            }


            //debugging
            //====================


            //====================
            //non-fieldCentric

            if (fieldCentric == false) {


                if(gamepad1.back) {//into field centric mode
                    fieldCentric = true;
                }


                //====================
                //non-fieldCentric variables

                double frontLeftPowerRaw = (y + x + (rx * -.7));
                double backLeftPowerRaw = (y - x + (rx * -.7));
                double frontRightPowerRaw = (y - x - (rx * -.7));
                double backRightPowerRaw = (y + x - (rx * -.7));

                //non-fieldCentric variables
                //====================

                if (!gamepad1.a) {//normal speed
                    left_front_drive.setPower(frontLeftPowerRaw * -.7);
                    left_back_drive.setPower(backLeftPowerRaw * -.7);
                    right_front_drive.setPower(frontRightPowerRaw * -.7);
                    right_back_drive.setPower(backRightPowerRaw * -.7);
                } else {
                    left_front_drive.setPower(frontLeftPowerRaw * -.3);
                    left_back_drive.setPower(backLeftPowerRaw * -.3);
                    right_front_drive.setPower(frontRightPowerRaw * -.3);
                    right_back_drive.setPower(backRightPowerRaw * -.3);
                }

                if ((frontLeftPowerRaw == 0) && (frontRightPowerRaw == 0) && (backRightPowerRaw == 0) && (backLeftPowerRaw == 0)) {
                    left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


                } else {
                    left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                }
            }//end non-fieldCentric if

            //non-fieldCentric
            //====================
            
            
            //====================
            //fieldCentric

            if (fieldCentric == true) {

                if (gamepad1.y == true) {//out of field centric mode
                    fieldCentric = false;
                }



                //====================
                //fieldCentric variables

                //IMU pull heading
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double orientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

                //rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                //fieldCentric variables
                //====================


                /*
                This button choice was made so that it is hard to hit on accident,
                it can be freely changed based on preference.
                The equivalent button is start on Xbox-style controllers.
                */
                if (gamepad1.b) {
                    imu.resetYaw();
                }

                rotX = rotX * 1.1;  //counteract imperfect strafing
                if (gamepad1.x) {
                    if (88 > orientation) {
                        right_front_drive.setPower(-0.7);
                        right_back_drive.setPower(-0.7);
                        left_back_drive.setPower(0.7);
                        left_front_drive.setPower(0.7);
                    } else if (110 < orientation) {
                        right_front_drive.setPower(0.7);
                        right_back_drive.setPower(0.7);
                        left_back_drive.setPower(-0.7);
                        left_front_drive.setPower(-0.7);
                    }

                    if (92 < orientation && 110 > orientation) {
                        right_front_drive.setPower(0.7);
                        right_back_drive.setPower(0.7);
                        left_back_drive.setPower(-0.7);
                        left_front_drive.setPower(-0.7);
                    } else if (88 > orientation && 70 < orientation) {
                        right_front_drive.setPower(-0.7);
                        right_back_drive.setPower(-0.7);
                        left_back_drive.setPower(0.7);
                        left_front_drive.setPower(0.7);
                    }

                    if (orientation < 92 && orientation > 88) {
                        right_front_drive.setPower(0);
                        right_back_drive.setPower(0);
                        left_back_drive.setPower(0);
                        left_front_drive.setPower(0);
                    }
                }

                /*
                Denominator is the largest motor power (absolute value) or 1
                This ensures all the powers maintain the same ratio,
                but only if at least one is out of the range [-1, 1]
                */
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                if (!gamepad1.a) {//regular field centric
                    left_front_drive.setPower(frontLeftPower);
                    left_back_drive.setPower(backLeftPower);
                    right_front_drive.setPower(frontRightPower);
                    right_back_drive.setPower(backRightPower);
                } else {//slowdown
                    left_front_drive.setPower(frontLeftPower * 0.3);
                    left_back_drive.setPower(backLeftPower * 0.3);
                    right_front_drive.setPower(frontRightPower * 0.3);
                    right_back_drive.setPower(backRightPower * 0.3);
                }
                if ((frontLeftPower == 0) && (frontRightPower == 0) && (backRightPower == 0) && (backLeftPower == 0)) {
                    left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


                } else {
                    left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                }

            }//end fieldCentric if
            
            //fieldCentric
            //====================


        }//ends while
    }//ends runOpMode
}//ends class
