package org.firstinspires.ftc.teamcode.drive.opmode.autos.red.close;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pretty.Globals;
import org.firstinspires.ftc.teamcode.pretty.hardware;
import org.firstinspires.ftc.teamcode.pretty.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.pretty.subsystems.WristSubsystem;

//@Autonomous
public class CloseLeftRed/* extends LinearOpMode*/ {
    hardware robot = new hardware();

    WristSubsystem wrist = new WristSubsystem();
    //@Override
    public void run/*OpMode*/(HardwareMap hardwareMap) {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        robot.init(hardwareMap, false);
        robot.runMode();

        //Claw Both Grab
        robot.clawL.setPosition(0);
        robot.clawR.setPosition(270);

        //waitForStart();


        //Pivot flat
        Globals.pivotNeutral();

        movePW();

        Trajectory toPurpleCenter1 = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(22, 2, Math.toRadians(90)))
                .build();
        drivetrain.followTrajectory(toPurpleCenter1);

        Trajectory pushProp = drivetrain.trajectoryBuilder(new Pose2d(24,2,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(22, 18, Math.toRadians(90)))
                .build();
        drivetrain.followTrajectory(pushProp);

        Trajectory toPurpleSpike = drivetrain.trajectoryBuilder(new Pose2d(22,18,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(22, 15.5, Math.toRadians(90)))
                .build();
        drivetrain.followTrajectory(toPurpleSpike);

        //Left Claw Release
        //sleep(250);
        //while (robot.clawL.getPosition() < Globals.CLAWLOPEN) {
            robot.clawL.setPosition(Globals.CLAWLOPEN);
        //}
        //sleep(250);

        Trajectory toBD1 = drivetrain.trajectoryBuilder(new Pose2d(22, 15.5, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    //Pivot up
                    Globals.pivotScoring();

                    movePW();
                })
                .lineToLinearHeading(new Pose2d(12, -22, Math.toRadians(90)))
                .build();
        drivetrain.followTrajectory(toBD1);

        Trajectory toBD2 = drivetrain.trajectoryBuilder(new Pose2d(12, -22, Math.toRadians(90)))

                .lineToLinearHeading(new Pose2d(12, -27, Math.toRadians(90)))
                .build();
        drivetrain.followTrajectory(toBD2);

        //Claw Right Release
        //sleep(250);
        //while (robot.clawR.getPosition() > Globals.CLAWROPEN) {
            robot.clawR.setPosition(Globals.CLAWROPEN);
        //}
        //sleep(250);

        Trajectory awayBD = drivetrain.trajectoryBuilder(new Pose2d(12,-27,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-10, -34, Math.toRadians(90)))
                .build();
        drivetrain.followTrajectory(awayBD);

        Trajectory toPark = drivetrain.trajectoryBuilder(new Pose2d(-10, -34, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    //Pivot flat
                    Globals.pivotNeutral();

                    movePW();
                })
                .lineToLinearHeading(new Pose2d(-10, -44, Math.toRadians(90)))
                .build();
        drivetrain.followTrajectory(toPark);
        //telemetry.addData("hehe", drivetrain.getPoseEstimate());



    }
    public void movePW() {

        Globals.readPositions(robot.pivotX.getCurrentPosition(), robot.pivotY.getCurrentPosition(), robot.slide.getCurrentPosition());

        PivotSubsystem.set();
        wrist.set();

        robot.pivotX.setTargetPosition((int) PivotSubsystem.xCalculate());
        robot.pivotY.setTargetPosition((int) PivotSubsystem.yCalculate());

        robot.wristX.setPosition(wrist.xCalculate());
        robot.wristY.setPosition(wrist.yCalculate());

    }
}
