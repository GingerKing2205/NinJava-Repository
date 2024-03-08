package org.firstinspires.ftc.teamcode.drive.opmode.autos.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pretty.Globals;
import org.firstinspires.ftc.teamcode.pretty.hardware;
import org.firstinspires.ftc.teamcode.pretty.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.pretty.subsystems.WristSubsystem;

@Autonomous
public class FarRightBlueBAD extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        hardware robot = new hardware();

        WristSubsystem wrist = new WristSubsystem();



        waitForStart();


        //Unfold
        Globals.pivotNeutral();
        PivotSubsystem.set();
        wrist.set();

        Trajectory toPurpleSpike = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(20, -12, 0))
                .build();
        drivetrain.followTrajectory(toPurpleSpike);

        //Left Claw Release
        robot.clawL.setPosition(Globals.CLAWLOPEN);

        //Pivot Top Stack Pos
        Globals.pivotStack = true;
        PivotSubsystem.set();
        wrist.set();


        Trajectory spikeToStack = drivetrain.trajectoryBuilder(new Pose2d(20,-12,0))
                .lineToLinearHeading(new Pose2d (20, -12, Math.toRadians(-60)))
                .build();
        drivetrain.followTrajectory(spikeToStack);

        Trajectory toStack = drivetrain.trajectoryBuilder(new Pose2d(22, 0, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d (24, -17, Math.toRadians(-60)))
                .build();
        drivetrain.followTrajectory(toStack);

        //Left Claw Grab
        robot.clawL.setPosition(Globals.CLAWLCLOSED);

        //Pivot stored
        Globals.pivotStack = false;
        Globals.pivotstored();
        PivotSubsystem.set();
        wrist.set();

        Trajectory trussAlign = drivetrain.trajectoryBuilder(new Pose2d(24, -17, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(4, 0, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(trussAlign);

        Trajectory throughTruss = drivetrain.trajectoryBuilder(new Pose2d(4, 0, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(4, 60, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(throughTruss);

        Trajectory toBD = drivetrain.trajectoryBuilder(new Pose2d(17, 60, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(30, 85, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(toBD);

        //Pivot up
        Globals.pivotScoring();
        PivotSubsystem.set();
        wrist.set();

        //Claw Both Release
        robot.clawL.setPosition(Globals.CLAWLOPEN);
        robot.clawR.setPosition(Globals.CLAWROPEN);

        //Pivot flat
        Globals.pivotNeutral();
        PivotSubsystem.set();
        wrist.set();

        Trajectory toBDFarSide = drivetrain.trajectoryBuilder(new Pose2d(30, 85, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(54, 85, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(toBDFarSide);

        telemetry.addData("hehe", drivetrain.getPoseEstimate());



    }

}
