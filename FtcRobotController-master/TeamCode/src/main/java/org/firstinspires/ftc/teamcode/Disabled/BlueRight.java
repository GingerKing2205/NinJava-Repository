package org.firstinspires.ftc.teamcode.Disabled;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.opmode.liftPID;

public class BlueRight {
    private liftPID arm = new liftPID();
    private IMU imu_IMU;
    private VoltageSensor ControlHub_VoltageSensor;
    private DcMotor right_back_drive;
    private DcMotor right_front_drive;
    private DcMotor Slide;
    private DcMotor Noose;
    private DcMotor left_back_drive;
    private DcMotor left_front_drive;
    private DcMotor intake;

    YawPitchRollAngles orientation;
    AngularVelocity angularVelocity;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    public void runOpMode(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException {

        arm.init(hardwareMap);
        boolean my_180complete;
        boolean my_90complete;

        imu_IMU = hardwareMap.get(IMU.class, "imu");
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        right_back_drive = hardwareMap.get(DcMotor.class, "right_back_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Noose = hardwareMap.get(DcMotor.class, "Noose");
        left_back_drive = hardwareMap.get(DcMotor.class, "left_back_drive");
        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");


        // Initialize the IMU.
        // Initialize the IMU with non-d
        // efault settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        // Prompt user to press start button.
        telemetry.addData("IMU Example", "Press start to continue...");
        telemetry.update();
        imu_IMU.resetYaw();
        if (ControlHub_VoltageSensor.getVoltage() < 12.5) {
            telemetry.addLine("CHARGE ME!!");
        }
        telemetry.update();
        right_back_drive.setDirection(DcMotor.Direction.REVERSE);
        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Noose.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Noose.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime runTime = new ElapsedTime();

        if (1 == 1) {
            // Put run blocks here.
            telemetry.addData("Right Side", "Detected");
            telemetry.addData("parallel odo steps", right_back_drive.getCurrentPosition());
            telemetry.addData("Yaw", "Press Circle or B on Gamepad to reset.");
            // Check to see if reset yaw is requested.
            // Get the orientation and angular velocity.
            orientation = imu_IMU.getRobotYawPitchRollAngles();
            angularVelocity = imu_IMU.getRobotAngularVelocity(AngleUnit.DEGREES);
            // Display yaw, pitch, and roll.

            telemetry.addData("Yaw (Z)", JavaUtil.formatNumber(orientation.getYaw(AngleUnit.DEGREES), 2));
            telemetry.addData("Pitch (X)", JavaUtil.formatNumber(orientation.getPitch(AngleUnit.DEGREES), 2));
            telemetry.addData("Roll (Y)", JavaUtil.formatNumber(orientation.getRoll(AngleUnit.DEGREES), 2));
            // Display angular velocity.
            telemetry.addData("Yaw (Z) velocity", JavaUtil.formatNumber(angularVelocity.zRotationRate, 2));
            telemetry.addData("Pitch (X) velocity", JavaUtil.formatNumber(angularVelocity.xRotationRate, 2));
            telemetry.addData("Roll (Y) velocity", JavaUtil.formatNumber(angularVelocity.yRotationRate, 2));
            telemetry.update();
            boolean fieldCentric = true; //fieldCentric mode
            int gallow = -3300;//lift to backdrop position
            int knot =0;//Noose intake position


            my_180complete = false;
            // Center zone
            reset_odo();
            while (right_back_drive.getCurrentPosition() < 6000) {
                left_back_drive.setPower(-0.4);
                left_front_drive.setPower(-0.4);
                right_back_drive.setPower(-0.4);
                right_front_drive.setPower(-0.4);
                telemetry.addData("para odo", right_back_drive.getCurrentPosition());
                telemetry.update();
            }
            left_back_drive.setPower(0);
            left_front_drive.setPower(0);
            right_back_drive.setPower(0);
            right_front_drive.setPower(0);
            reset_odo();
            my_90complete = false;
            DT_power_0();
            while (false == my_90complete) {
                imu_update();
                if (orientation.getYaw(AngleUnit.DEGREES) < 70 || orientation.getYaw(AngleUnit.DEGREES) < -1) {
                    left_back_drive.setPower(-0.6);
                    left_front_drive.setPower(-0.6);
                    right_back_drive.setPower(0.6);
                    right_front_drive.setPower(0.6);
                } else if (orientation.getYaw(AngleUnit.DEGREES) < 90 || orientation.getYaw(AngleUnit.DEGREES) < -1) {
                    left_back_drive.setPower(-0.2);
                    left_front_drive.setPower(-0.2);
                    right_back_drive.setPower(0.2);
                    right_front_drive.setPower(0.2);
                } else {
                    left_back_drive.setPower(0);
                    left_front_drive.setPower(0);
                    right_back_drive.setPower(0);
                    right_front_drive.setPower(0);
                    my_90complete = true;
                }
            }
            reset_odo();
            //back past x
            while (right_back_drive.getCurrentPosition() < 3500) {
                left_back_drive.setPower(-0.3);
                left_front_drive.setPower(-0.3);
                right_back_drive.setPower(-0.3);
                right_front_drive.setPower(-0.3);
                telemetry.addData("para odo", right_back_drive.getCurrentPosition());
                telemetry.update();
            }
            DT_power_0();
            my_90complete = false;
            while (false == my_90complete) {
                imu_update();
                if (orientation.getYaw(AngleUnit.DEGREES) < 70 || orientation.getYaw(AngleUnit.DEGREES) < -1) {
                    left_back_drive.setPower(-0.6);
                    left_front_drive.setPower(-0.6);
                    right_back_drive.setPower(0.6);
                    right_front_drive.setPower(0.6);
                } else if (orientation.getYaw(AngleUnit.DEGREES) < 90 || orientation.getYaw(AngleUnit.DEGREES) < -1) {
                    left_back_drive.setPower(-0.2);
                    left_front_drive.setPower(-0.2);
                    right_back_drive.setPower(0.2);
                    right_front_drive.setPower(0.2);
                } else {
                    left_back_drive.setPower(0);
                    left_front_drive.setPower(0);
                    right_back_drive.setPower(0);
                    right_front_drive.setPower(0);
                    my_90complete = true;
                }
            }

            reset_odo();
            while (!(left_back_drive.getCurrentPosition() <= -46000)) {//changed from -48000
                left_back_drive.setPower(0.6);
                left_front_drive.setPower(-0.6);
                right_back_drive.setPower(-0.6);
                right_front_drive.setPower(0.6);
                telemetry.addData("horizontal odo", left_back_drive.getCurrentPosition());
                telemetry.update();
            }
            DT_power_0();
            reset_odo();
            my_90complete = false;
            while (false == my_90complete) {
                imu_update();
                if (orientation.getYaw(AngleUnit.DEGREES) < 70 || orientation.getYaw(AngleUnit.DEGREES) < -1) {
                    left_back_drive.setPower(-0.6);
                    left_front_drive.setPower(-0.6);
                    right_back_drive.setPower(0.6);
                    right_front_drive.setPower(0.6);
                } else if (orientation.getYaw(AngleUnit.DEGREES) < 90 || orientation.getYaw(AngleUnit.DEGREES) < -1) {
                    left_back_drive.setPower(-0.2);
                    left_front_drive.setPower(-0.2);
                    right_back_drive.setPower(0.2);
                    right_front_drive.setPower(0.2);
                } else {
                    left_back_drive.setPower(0);
                    left_front_drive.setPower(0);
                    right_back_drive.setPower(0);
                    right_front_drive.setPower(0);
                    my_90complete = true;
                }
            }
            //
            while (!(Noose.getCurrentPosition() <= -550)) {
                Noose.setPower(-1);
            }
            Noose.setPower(0);
            while (!(Slide.getCurrentPosition() >= 580)) {
                Slide.setPower(0.5);
            }
            Slide.setPower(0);

            while (!(Slide.getCurrentPosition() <= 370)) {
                Slide.setPower(-0.5);
            }
            Slide.setPower(0);
            while (!(Noose.getCurrentPosition() >= -1000)) {
                Noose.setPower(1);
            }
            Noose.setPower(0);
            Noose.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Noose.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setPower(0.4);
            sleep(1000);
            intake.setPower(0);
            while (!(Noose.getCurrentPosition() >= 700)) {
                Noose.setPower(1);
            }
            Noose.setPower(0);
            while (!(Slide.getCurrentPosition() <= 500)) {
                Slide.setPower(-0.5);
            }
            while (!(Noose.getCurrentPosition() <= 50)) {
                Noose.setPower(-1);
            }
            Noose.setPower(0);
            Noose.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Noose.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            DT_power_0();
            my_90complete = false;
            while (false == my_90complete) {
                imu_update();
                if (orientation.getYaw(AngleUnit.DEGREES) < 70 || orientation.getYaw(AngleUnit.DEGREES) < -1) {
                    left_back_drive.setPower(-0.6);
                    left_front_drive.setPower(-0.6);
                    right_back_drive.setPower(0.6);
                    right_front_drive.setPower(0.6);
                } else if (orientation.getYaw(AngleUnit.DEGREES) < 90 || orientation.getYaw(AngleUnit.DEGREES) < -1) {
                    left_back_drive.setPower(-0.2);
                    left_front_drive.setPower(-0.2);
                    right_back_drive.setPower(0.2);
                    right_front_drive.setPower(0.2);
                } else {
                    left_back_drive.setPower(0);
                    left_front_drive.setPower(0);
                    right_back_drive.setPower(0);
                    right_front_drive.setPower(0);
                    my_90complete = true;
                }
            }
            my_90complete = true; //setup for alignment later**
            while (false == my_90complete) {
                imu_update();
                if (orientation.getYaw(AngleUnit.DEGREES) > 90) {
                    left_back_drive.setPower(-0.2);
                    left_front_drive.setPower(-0.2);
                    right_back_drive.setPower(0.2);
                    right_front_drive.setPower(0.2);
                } else {
                    left_back_drive.setPower(0);
                    left_front_drive.setPower(0);
                    right_back_drive.setPower(0);
                    right_front_drive.setPower(0);
                    my_90complete = true;
                }
            }
            //slight left strafe
            reset_odo();
            while (!(left_back_drive.getCurrentPosition() <= -3200)) {
                left_back_drive.setPower(0.6);
                left_front_drive.setPower(-0.6);
                right_back_drive.setPower(-0.6);
                right_front_drive.setPower(0.6);
                telemetry.addData("horizontal odo", left_back_drive.getCurrentPosition());
                telemetry.update();
            }
            DT_power_0();
            left_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_back_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            reset_odo();
            runTime.reset();
            while (right_back_drive.getCurrentPosition() < 63000 && runTime.seconds() < 4) {
                left_back_drive.setPower(-0.4);
                left_front_drive.setPower(-0.4);
                right_back_drive.setPower(-0.4);
                right_front_drive.setPower(-0.4);
                telemetry.addData("para odo", right_back_drive.getCurrentPosition());
                telemetry.update();
            }
            DT_power_0();
            reset_odo();
            boolean armOn = true;
            boolean scored = false;

            Noose.setTargetPosition(550);//lifts noose
            Noose.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.reset();
            arm.target = -3600;//raises arm
            while (armOn == true){
                double FlipStep = arm.leftFlip.getCurrentPosition();
                arm.loop();
                ((DcMotorEx) Noose).setVelocity(4000);


                if (FlipStep < -3200){//flips and deposits
                    if (Noose.getCurrentPosition() >= 545){
                        sleep(750);
                        arm.target = 0;
                        scored = true; //added

                        left_back_drive.setPower(-0.8);
                        left_front_drive.setPower(0.8);
                        right_back_drive.setPower(0.8);
                        right_front_drive.setPower(-0.8);
                        telemetry.addData("horizontal odo", left_back_drive.getCurrentPosition());
                        telemetry.update();
                    }
                    else {
                        Noose.setTargetPosition(knot + 550);

                    }
                } else {
                    Noose.setTargetPosition((int) (FlipStep * 0.545 + knot + 550));
                }

                if (scored && FlipStep > -200) {
                    armOn = false;
                }

                telemetry.addData("Arm Target", arm.target);
                telemetry.update();
            }
            telemetry.addData("Exited Loop!", true);
            telemetry.update();
            arm.target = 0;
            arm.kill();

            /*while (!(left_back_drive.getCurrentPosition() >= 50000)) {//strafe left
                left_back_drive.setPower(-0.8);
                left_front_drive.setPower(0.8);
                right_back_drive.setPower(0.8);
                right_front_drive.setPower(-0.8);
                telemetry.addData("horizontal odo", left_back_drive.getCurrentPosition());
                telemetry.update();
            }*/
            Noose.setTargetPosition(knot);
            DT_power_0();

            sleep(100000);
        }
    }

    /**
     * Describe this function...
     */
    private void reset_odo() throws InterruptedException {
        right_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
        right_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * Describe this function...
     */
    private void imu_update() {
        orientation = imu_IMU.getRobotYawPitchRollAngles();
        angularVelocity = imu_IMU.getRobotAngularVelocity(AngleUnit.DEGREES);
    }

    /**
     * Describe this function...
     */
    private void DT_power_0() {
        left_back_drive.setPower(0);
        left_front_drive.setPower(0);
        right_back_drive.setPower(0);
        right_front_drive.setPower(0);
    }
}