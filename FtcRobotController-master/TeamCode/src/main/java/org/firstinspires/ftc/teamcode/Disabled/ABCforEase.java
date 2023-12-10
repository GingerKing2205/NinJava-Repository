package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;





    @TeleOp(name = "ABCforEase")
    @Disabled
    public class ABCforEase extends LinearOpMode {

        private DcMotor Slide;
        private DcMotor Noose;
        private DcMotor leftFlip;
        private DcMotor left_back_drive;
        private DcMotor left_front_drive;
        private DcMotor right_back_drive;
        private DcMotor right_front_drive;
        private DcMotor intake;
        private Servo plane;

        /**
         * This function is executed when this OpMode is selected from the Driver Station.
         */
        @Override
        public void runOpMode() {
            int FlipStep;
            float y_axis;
            float turn;
            float x_axis;

            Slide = hardwareMap.get(DcMotor.class, "Slide");
            Noose = hardwareMap.get(DcMotor.class, "Noose");
            leftFlip = hardwareMap.get(DcMotor.class, "leftFlip");
            left_back_drive = hardwareMap.get(DcMotor.class, "left_back_drive");
            left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
            right_back_drive = hardwareMap.get(DcMotor.class, "right_back_drive");
            right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
            intake = hardwareMap.get(DcMotor.class, "intake");
            plane = hardwareMap.get(Servo.class, "plane");

            // Put initialization blocks here.
            Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Noose.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slide.setTargetPosition(0);
            leftFlip.setTargetPosition(0);
            Noose.setTargetPosition(550);
            left_back_drive.setDirection(DcMotor.Direction.REVERSE);
            left_front_drive.setDirection(DcMotor.Direction.REVERSE);
            left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Noose.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Noose.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            waitForStart();


            if (opModeIsActive()) { //op mode start
                while (opModeIsActive()) {
                    FlipStep = leftFlip.getCurrentPosition();
                    y_axis = gamepad1.left_stick_y;
                    turn = -gamepad1.right_stick_x;
                    x_axis = gamepad1.left_stick_x;
                    // Put loop blocks here.
                    telemetry.update();
                    telemetry.addData("slide", Slide.getCurrentPosition());
                    telemetry.addData("Noose", Noose.getCurrentPosition());
                    telemetry.addData("NooseTarget", Noose.getTargetPosition());
                    telemetry.addData("NoosePower", Noose.getPower());
                    telemetry.addData("LeftFlip", leftFlip.getCurrentPosition());
                    ((DcMotorEx) leftFlip).setVelocity(7000);
                    ((DcMotorEx) Noose).setVelocity(3000);
                    ((DcMotorEx) Slide).setVelocity(2000);
                    right_back_drive.setPower((y_axis - turn) - x_axis);
                    right_front_drive.setPower((y_axis - turn) + x_axis);
                    left_front_drive.setPower((y_axis + turn) - x_axis);
                    left_back_drive.setPower(y_axis + turn + x_axis);

                    if (gamepad1.a == true) {//backs robot
                        right_back_drive.setPower(0.3);
                        right_front_drive.setPower(0.3);
                        left_front_drive.setPower(0.3);
                        left_back_drive.setPower(0.3);
                    }

                    if (gamepad1.right_bumper == true) {//strafe right
                        right_back_drive.setPower(-0.4);
                        right_front_drive.setPower(0.4);
                        left_front_drive.setPower(-0.4);
                        left_back_drive.setPower(0.4);
                    } else if (gamepad1.left_bumper == true) {//strafe left
                        right_back_drive.setPower(0.4);
                        right_front_drive.setPower(-0.4);
                        left_front_drive.setPower(0.4);
                        left_back_drive.setPower(-0.4);
                    }

                    if (gamepad1.right_trigger > 0) {//turn right
                        right_back_drive.setPower(0.4);
                        right_front_drive.setPower(0.4);
                        left_front_drive.setPower(-0.4);
                        left_back_drive.setPower(-0.4);
                    } else if (gamepad1.left_trigger > 0) {//turn left
                        right_back_drive.setPower(-0.4);
                        right_front_drive.setPower(-0.4);
                        left_front_drive.setPower(0.4);
                        left_back_drive.setPower(0.4);
                    }

                    if (gamepad2.right_trigger > 0) {//extends slide
                        Slide.setTargetPosition(1300);
                    } else if (gamepad2.left_trigger > 0) {//retracts slide
                        Slide.setTargetPosition(0);
                    } else {//holds slides
                        Slide.setTargetPosition(Slide.getCurrentPosition());
                    }

                    if (gamepad2.y == true) {//raises Flip
                        leftFlip.setTargetPosition(-3300);
                        Noose.setTargetPosition((int) (FlipStep * 0.545 + 550));//Intake-Ground Parallel
                        if (leftFlip.getCurrentPosition() < -2200) {//slows Flip
                            ((DcMotorEx) leftFlip).setVelocity(500);
                        } else {
                            ((DcMotorEx) Noose).setVelocity(3000);
                        }
                    }

                    if (gamepad2.x == true) {//lowers flip
                        Noose.setTargetPosition(550);
                        leftFlip.setTargetPosition(0);
                        if (leftFlip.getCurrentPosition() > -2200) {//slows Flip
                            ((DcMotorEx) leftFlip).setVelocity(800);
                        }
                    }

                    if (gamepad2.left_bumper == true) {//Intake-Flip parallel
                        Noose.setTargetPosition(550);
                    }

                    if (gamepad2.right_bumper == true) {//lowers intake
                        Noose.setTargetPosition(0);
                    }

                    if (gamepad2.left_stick_y != 0) {//intake power
                        intake.setPower(gamepad2.left_stick_y);
                    } else {
                        intake.setPower(0);
                    }

                    if (gamepad2.start == true) {//plane launcher
                        plane.setPosition(-270);
                    }

                    if (gamepad2.dpad_right == true) {//lift encoder reset
                        leftFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        leftFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }

                    if (gamepad2.dpad_up == true) {//manual lift up
                        leftFlip.setTargetPosition(leftFlip.getCurrentPosition() - 50);
                    }

                    if (gamepad2.dpad_down == true) {//manual lift down
                        leftFlip.setTargetPosition(leftFlip.getCurrentPosition() + 50);
                    }

                    if (gamepad2.a == true) {//killswitch
                        terminateOpModeNow();
                        for (int count = 0; count < 10; count++) {
                        }
                    }
                }//ends while loop
            }//ends op mode
        }//ends public void
    }//ends extension
