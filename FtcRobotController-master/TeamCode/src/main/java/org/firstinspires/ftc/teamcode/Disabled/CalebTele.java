package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Disabled
public class CalebTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("left_front_drive");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("left_back_drive");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("right_front_drive");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("right_back_drive");
        DcMotor FlippityFlop = hardwareMap.dcMotor.get("leftFlip");
        //DcMotor Noose = hardwareMap.dcMotor.get("Noose");
        DcMotor IP = hardwareMap.dcMotor.get("Noose");
        DcMotor Noods = hardwareMap.dcMotor.get("intake");
        // Servo Intake = hardwareMap.servo.get("intakeFlip");
        Servo Drone = hardwareMap.servo.get("plane");
        FlippityFlop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FlippityFlop.setTargetPosition(0);
        FlippityFlop.setMode(DcMotor.RunMode.RUN_TO_POSITION);// <-----------------------
        IP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IP.setTargetPosition(-50);

        telemetry.addData("pivot steps", FlippityFlop.getCurrentPosition());

        // DcMotor rightFlipMotor = hardwareMap.dcMotor.get("rightFlip");
        // Rev2mDistanceSensor distance = hardwareMap.get(Rev2mDistanceSensor.class, "distance");
        //frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotor Slide = hardwareMap.dcMotor.get("Slide");
        boolean fieldCentric = true;
        boolean Hang = false;
        boolean Retracting = false;
        boolean vert = false;
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        // frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x; //i made this neg to try and fix turning

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.b) {
                imu.resetYaw();
            }



            telemetry.addData("fieldCentric", fieldCentric);



            if (fieldCentric == false){
                //if (IP.getCurrentPosition() == IP.getTargetPosition()){
                //   IP.setPower(1);
                // }
                // else if (IP.getTargetPosition() > IP.getCurrentPosition()){
                //     IP.setPower(1);
                /// }
                // else{
                //     IP.setPower(-1);
                // }
                telemetry.addData("pivot steps", FlippityFlop.getCurrentPosition());
                // telemetry.addData("intake steps", Intake.getPosition());
                telemetry.addData("intake angle", IP.getCurrentPosition());
                telemetry.update();

                if (gamepad1.dpad_up){
                    Noods.setPower(1);
                }
                else if (gamepad1.dpad_down){
                    Noods.setPower(-1);
                }

                if (gamepad2.x){
                    Hang = true;

                }
                if (Hang == true) {
                    // Intake.setPosition(0);

                    if (gamepad2.right_stick_button)  {
                        Hang = false;

                    }
                    if (gamepad2.y){
                        vert = true;

                    }
                    if (vert == true){

                        // Intake.setDirection(Servo.Direction.REVERSE);
                        FlippityFlop.setTargetPosition(-1392);
                        FlippityFlop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        FlippityFlop.setPower(1);
                        FlippityFlop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                        if (FlippityFlop.getTargetPosition() == FlippityFlop.getCurrentPosition()) {
                            FlippityFlop.setPower(0);
                        }
                    }
                    if (gamepad2.dpad_up) {
                        FlippityFlop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        FlippityFlop.setPower(1); //
                        // rightFlipMotor.setPower(-1);
                    } else if (gamepad2.dpad_down) {
                        FlippityFlop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        FlippityFlop.setPower(-1); //was 1
                        //rightFlipMotor.setPower(1);

                    } else if (gamepad2.start){
                        Drone.setPosition(-270);
                    }
                    else if (gamepad2.back){
                        Drone.setPosition(Drone.getPosition() + 270);
                    }
                    else {
                        FlippityFlop.setPower(0);
                        //  rightFlipMotor.setPower(0);
                    }
                }
                if (Hang == false) {
                    if (gamepad2.right_bumper && FlippityFlop.getCurrentPosition() > -100){
                        Noods.setPower(1);
                        //Intake.setDirection(Servo.Direction.REVERSE); //
                        //Intake.setPosition(0.6);// IM GONNA KILL MYSELF THERE IS NO WAY
                        IP.setTargetPosition(0);
                    }
                    else if (gamepad2.y) {
                        // Intake.setDirection(Servo.Direction.REVERSE);
                        FlippityFlop.setTargetPosition(-1792);
                        FlippityFlop.setPower(.75);
                        FlippityFlop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        if (FlippityFlop.getCurrentPosition() < -400){

                            if (gamepad2.right_bumper){

                            }
                            else {
                                // Intake.setPosition(.8);
                                IP.setTargetPosition(30);
                            }
                            if (FlippityFlop.getCurrentPosition() < -1000){

                                if (gamepad2.right_bumper){

                                }
                                else {
                                    // Intake.setPosition(.9);
                                    IP.setTargetPosition(55);
                                }
                            }
                            if (FlippityFlop.getCurrentPosition() < -1600){

                                if (gamepad2.right_bumper){
                                    //Intake.setPosition(.45);
                                    IP.setTargetPosition(-50);
                                }
                                else {
                                    // Intake.setPosition(.9);
                                    IP.setTargetPosition(80);
                                }
                            }
                        }
                        if (FlippityFlop.getTargetPosition() == FlippityFlop.getCurrentPosition()) {
                            FlippityFlop.setPower(0);
                        }


                    } else {
                        FlippityFlop.setTargetPosition(40);
                        FlippityFlop.setPower(-0.4);
                        //Intake.setDirection(Servo.Direction.FORWARD); //was forward
                        Noods.setPower(0);
                        // Intake.setPosition(0.7);// was 140
                        IP.setTargetPosition(-50);
                        if (FlippityFlop.getCurrentPosition() == FlippityFlop.getTargetPosition()) {
                            FlippityFlop.setPower(0);
                            FlippityFlop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        }

                    }

                }
                if (gamepad1.y){

                    fieldCentric = false;
                }

                if (gamepad2.left_trigger>0) {

                    Slide.setPower(-1);

                }
                else if (gamepad2.right_trigger>0){
                    Slide.setPower(1);
                }
                else{
                    Slide.setPower(0);
                }
                //if (Slide.getCurrentPosition() < Slide.getTargetPosition() && Retracting == true){
                // Slide.setTargetPosition(0);
                // Slide.setPower(-1);
                // Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                /// Slide.setPower(-1);

                //}




                //Noose.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                if (gamepad2.a){
                    //Noose.setPower(100);
                }
                else if (gamepad2.b){
                    //Noose.setPower(-100);

                }
                else{
                    //Noose.setPower(0);
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double orientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                FlippityFlop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                // rightFlipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                rotX = rotX * 1.1;  // Counteract imperfect strafing
                if (gamepad1.a) {
                    if (88 > orientation && 70 > orientation) {
                        frontRightMotor.setPower(-0.7);
                        backRightMotor.setPower(-0.7);
                        backLeftMotor.setPower(0.7);
                        frontLeftMotor.setPower(0.7);
                    } else if (92 < orientation && 110 < orientation) {
                        frontRightMotor.setPower(0.7);
                        backRightMotor.setPower(0.7);
                        backLeftMotor.setPower(-0.7);
                        frontLeftMotor.setPower(-0.7);
                    }
                    if (92 < orientation && 110 > orientation) {
                        frontRightMotor.setPower(0.7);
                        backRightMotor.setPower(0.7);
                        backLeftMotor.setPower(-0.7);
                        frontLeftMotor.setPower(-0.7);
                    } else if (88 > orientation && 70 < orientation) {
                        frontRightMotor.setPower(-0.7);
                        backRightMotor.setPower(-0.7);
                        backLeftMotor.setPower(0.7);
                        frontLeftMotor.setPower(0.7);
                    }
                    if (orientation < 92 && orientation > 88) {
                        frontRightMotor.setPower(0);
                        backRightMotor.setPower(0);
                        backLeftMotor.setPower(0);
                        frontLeftMotor.setPower(0);
                    }
                }

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPowerRaw = (y + x + (rx * -.7));
                double backLeftPowerRaw = (y - x + (rx * -.7));
                double frontRightPowerRaw = (y - x - (rx * -.7));
                double backRightPowerRaw = (y + x - (rx * -.7));

                frontLeftMotor.setPower(frontLeftPowerRaw * -.7);
                backLeftMotor.setPower(backLeftPowerRaw * -.7);
                frontRightMotor.setPower(frontRightPowerRaw * -.7);
                backRightMotor.setPower(backRightPowerRaw * -.7);

                if ((frontLeftPowerRaw == 0) && (frontRightPowerRaw == 0) && (backRightPowerRaw == 0) && (backLeftPowerRaw == 0)) {
                    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


                }
                else{
                    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                }

            }
            if (fieldCentric == true) {
                if (IP.getCurrentPosition() == IP.getTargetPosition()){
                    IP.setPower(0);
                }
                else if (IP.getTargetPosition() > IP.getCurrentPosition()){
                    IP.setPower(.3);
                }
                else{
                    IP.setPower(-.3);
                }
                telemetry.addData("pivot steps", FlippityFlop.getCurrentPosition());
                //telemetry.addData("intake steps", Intake.getPosition());
                telemetry.addData("intake rotation", IP.getCurrentPosition());
                telemetry.update();

                if (gamepad1.dpad_up){
                    Noods.setPower(1);
                }
                else if (gamepad1.dpad_down){
                    Noods.setPower(-1);
                }

                if (gamepad2.x && Hang==false){
                    Hang = true;

                }
                if (Hang == true) {
                    if (gamepad2.y){
                        vert = true;
                    }
                    if (gamepad2.right_stick_button)  {
                        Hang = false;

                    }
                    // Intake.setPosition(1);
                    if (gamepad2.y){// was if vert==true
                        // Intake.setDirection(Servo.Direction.REVERSE);
                        FlippityFlop.setTargetPosition(-1392);
                        FlippityFlop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        FlippityFlop.setPower(.85);
                        FlippityFlop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                        if (FlippityFlop.getTargetPosition() == FlippityFlop.getCurrentPosition()) {
                            FlippityFlop.setPower(0);
                        }
                    }
                    if (gamepad2.dpad_up) {
                        FlippityFlop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        FlippityFlop.setPower(1); //
                        // rightFlipMotor.setPower(-1);
                    } else if (gamepad2.dpad_down) {
                        FlippityFlop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        FlippityFlop.setPower(-1); //was 1
                        //rightFlipMotor.setPower(1);

                    } else if (gamepad2.start){
                        Drone.setPosition(-270);
                    }
                    else if (gamepad2.back){
                        Drone.setPosition(Drone.getPosition() + 270);
                    }
                    else {
                        FlippityFlop.setPower(0);
                        //  rightFlipMotor.setPower(0);
                    }
                }
                if (Hang == false) {

                    if (FlippityFlop.getCurrentPosition() > -100){
                        if (gamepad2.right_bumper){
                            Noods.setPower(1);
                            //Intake.setDirection(Servo.Direction.REVERSE); //
                            IP.setTargetPosition(0);
                            // Intake.setPosition(0.6);// IM GONNA KILL MYSELF THERE IS NO WAY

                        }
                        else{
                            IP.setTargetPosition(-50);
                            Noods.setPower(0);
                        }

                    }
                    // if (gamepad2.right_bumper && FlippityFlop.getCurrentPosition() > -100){
                    //  Noods.setPower(1);
                    //Intake.setDirection(Servo.Direction.REVERSE); //
                    // IP.setTargetPosition(0);
                    // Intake.setPosition(0.6);// IM GONNA KILL MYSELF THERE IS NO WAY

                    // }
                    else if (gamepad2.y) {
                        //Intake.setDirection(Servo.Direction.REVERSE);
                        FlippityFlop.setTargetPosition(-1792);
                        FlippityFlop.setPower(.75);
                        FlippityFlop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        if (FlippityFlop.getCurrentPosition() < -400){

                            if (gamepad2.right_bumper){

                            }
                            else {
                                // Intake.setPosition(.8);
                                IP.setTargetPosition(30);
                            }
                            if (FlippityFlop.getCurrentPosition() < -1000){

                                if (gamepad2.right_bumper){

                                }
                                else {
                                    // Intake.setPosition(.9);
                                    IP.setTargetPosition(55);
                                }
                            }
                            if (FlippityFlop.getCurrentPosition() < -1600){

                                if (gamepad2.right_bumper){
                                    // Intake.setPosition(.45);
                                    IP.setTargetPosition(-50);
                                }
                                else {
                                    // Intake.setPosition(.9);
                                    IP.setTargetPosition(80);
                                }
                            }
                        }
                        if (FlippityFlop.getTargetPosition() == FlippityFlop.getCurrentPosition()) {
                            FlippityFlop.setPower(0);
                        }


                    } else {
                        FlippityFlop.setTargetPosition(40);
                        FlippityFlop.setPower(-0.4);
                        //Intake.setDirection(Servo.Direction.FORWARD); //was forward
                        Noods.setPower(0);
                        //Intake.setPosition(0.7);// was 140

                        IP.setTargetPosition(-50);
                        // if (IP.getCurrentPosition() > -50){
                        //    IP.setPower(-.3);
                        // }
                        //  else if (IP.getCurrentPosition() == 50) {
                        //      IP.setPower(0);
                        //}
                        //else{
                        //  IP.setPower(.3);
                        //}
                        if (FlippityFlop.getCurrentPosition() == FlippityFlop.getTargetPosition()) {
                            FlippityFlop.setPower(0);
                            FlippityFlop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        }

                    }

                }
                if (gamepad1.y){

                    fieldCentric = false;
                }
                //   if (Retracting == false && gamepad2.right_trigger>0){
                //       Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //      Slide.setPower(1);

                // }
                if (gamepad2.left_trigger>0) {

                    Slide.setPower(-1);

                }
                else if (gamepad2.right_trigger>0){
                    Slide.setPower(1);
                }
                else{
                    Slide.setPower(0);
                }

                /*====================
                else if (Retracting == false) {
                   Slide.setPower(0);
                 }
                if (Slide.getCurrentPosition() < Slide.getTargetPosition() && Retracting == true){
                   Slide.setTargetPosition(0);
                 Slide.setPower(-1);
                 Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 Slide.setPower(-1);

                }
                 else if (gamepad2.right_trigger==0) {
                  Slide.setPower(0);

                }
                ====================*/



                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double orientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                //Noose.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                if (gamepad2.a){
                    //Noose.setPower(100);
                }
                else if (gamepad2.b){
                    //Noose.setPower(-100);

                }
                else{
                    //Noose.setPower(0);
                }
                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                FlippityFlop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                // rightFlipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                rotX = rotX * 1.1;  // Counteract imperfect strafing
                if (gamepad1.a) {
                    if (88 > orientation && 70 > orientation) {
                        frontRightMotor.setPower(-0.7);
                        backRightMotor.setPower(-0.7);
                        backLeftMotor.setPower(0.7);
                        frontLeftMotor.setPower(0.7);
                    } else if (92 < orientation && 110 < orientation) {
                        frontRightMotor.setPower(0.7);
                        backRightMotor.setPower(0.7);
                        backLeftMotor.setPower(-0.7);
                        frontLeftMotor.setPower(-0.7);
                    }
                    if (92 < orientation && 110 > orientation) {
                        frontRightMotor.setPower(0.7);
                        backRightMotor.setPower(0.7);
                        backLeftMotor.setPower(-0.7);
                        frontLeftMotor.setPower(-0.7);
                    } else if (88 > orientation && 70 < orientation) {
                        frontRightMotor.setPower(-0.7);
                        backRightMotor.setPower(-0.7);
                        backLeftMotor.setPower(0.7);
                        frontLeftMotor.setPower(0.7);
                    }
                    if (orientation < 92 && orientation > 88) {
                        frontRightMotor.setPower(0);
                        backRightMotor.setPower(0);
                        backLeftMotor.setPower(0);
                        frontLeftMotor.setPower(0);
                    }
                }

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;


                // if (frontLeftMotor.getCurrentPosition() < 50){
                // frontRightMotor.setPower(frontRightPower * 0.2);
                // frontLeftMotor.setPower(frontLeftPower * 0.2);
                // backRightMotor.setPower(backRightPower * 0.2);
                // backLeftMotor.setPower(backLeftPower * 0.2);

                // }


                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
                if ((frontLeftPower == 0) && (frontRightPower == 0) && (backRightPower == 0) && (backLeftPower == 0)) {
                    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


                }
                else{
                    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                }
            }


        }
    }
}
