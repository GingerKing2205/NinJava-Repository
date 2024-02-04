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
public class ShowcaseMode extends LinearOpMode {

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
        //assign motors/servos

        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Noose = hardwareMap.get(DcMotor.class, "Noose");
        left_back_drive = hardwareMap.get(DcMotor.class, "left_back_drive");
        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        right_back_drive = hardwareMap.get(DcMotor.class, "right_back_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
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

        boolean nextReleased = true;
        int showStep = 0;

        int gallow = -3300;//lift to backdrop position
        boolean downReleased = true;
        boolean upReleased = true;

        int knot = 0;//Noose intake position

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
            //showcase

            //step 1: Lift, Score, and Return Pivot
            if (showStep == 0) {
                boolean complete1 = false;
                boolean complete2 = false;
                boolean complete3 = false;

                //lift
                if (!complete1) {
                    arm.target = -3300;
                    Noose.setTargetPosition((int) (FlipStep * 0.545 + knot + 550));//Intake-Ground Parallel
                }
                if (arm.leftFlip.getCurrentPosition() <= -3200
                    || arm.leftFlip.getCurrentPosition() >= -3400) {
                    complete1 = true;
                }

                //score
                if (!complete2) {
                    Noose.setTargetPosition(550);
                }
                if (Noose.getCurrentPosition() <= 560
                    || Noose.getCurrentPosition() >= 540) {
                    complete2 = true;
                }

                //return
                if (!complete3) {
                    arm.target = 0;
                    Noose.setTargetPosition((int) (FlipStep * 0.545 + knot + 550));//Intake-Ground Parallel
                }
                if (arm.leftFlip.getCurrentPosition() >= -100
                    || arm.leftFlip.getCurrentPosition() <= 100) {
                    complete3 = true;
                    showStep++;
                }

            }

            //step 2: Extend and Retract
            if (showStep == 1 && gamepad1.a) {
                showStep++;

                while (Slide.getCurrentPosition() < 800) Slide.setPower(1);
                while (Slide.getCurrentPosition() > 10) Slide.setPower(-1);
            }

            //step 3: Lift Pivot
            if (showStep == 3 && gamepad1.a) {
                arm.target = -3300;
                Noose.setTargetPosition((int) (FlipStep * 0.545 + knot + 550));//Intake-Ground Parallel
                if (arm.leftFlip.getCurrentPosition() <= -3200
                    || arm.leftFlip.getCurrentPosition() >= -3400) {
                    showStep++;
                }
            }

            //step 4: Deploy Hang
            if (showStep == 4 && gamepad1.a) {
                hang.setPower(.5);
                sleep(500);
                hang.setPower(0);
                showStep++;
            }

            //step 5: Drop Pivot
            if (showStep == 5) {
                arm.target = 0;
                Noose.setTargetPosition((int) (FlipStep * 0.545 + knot + 550));//Intake-Ground Parallel
                if (arm.leftFlip.getCurrentPosition() <= 100
                        || arm.leftFlip.getCurrentPosition() >= -100) {
                    showStep++;
                }
            }

            //step 6: kys
            if (showStep == 6) {//killswitch
                terminateOpModeNow();
                for (int count = 0; count < 10; count++) {
                }
            }

            //showcase
            //====================


            //====================
            //debugging

            if (gamepad2.a == true) {//killswitch
                terminateOpModeNow();
                for (int count = 0; count < 10; count++) {
                }
            }

            //debugging
            //====================


        }//ends while
    }//ends runOpMode
}//ends class
