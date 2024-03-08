package org.firstinspires.ftc.teamcode.drive.opmode.autos.red.far;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pretty.Globals;
import org.firstinspires.ftc.teamcode.pretty.hardware;
import org.firstinspires.ftc.teamcode.pretty.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.pretty.subsystems.WristSubsystem;

//@Autonomous
public class FarLeftRed /*extends LinearOpMode*/ {

    public hardware robot = new hardware();

    public WristSubsystem wrist = new WristSubsystem();

    //@Override
    public void run/*OpMode*/(HardwareMap hardwareMap) {

        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        robot.init(hardwareMap, false);

        robot.runMode();

        robot.clawL.setPosition(0);
        robot.clawR.setPosition(270);

        //waitForStart();


        //Unfold
        Globals.pivotNeutral();

        movePW();

        Trajectory toPurpleSpike = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(15, 6, 0))
                .build();
        drivetrain.followTrajectory(toPurpleSpike);

        //Left Claw Release
        //sleep(250);
        //while (robot.clawR.getPosition() > Globals.CLAWROPEN) {
            robot.clawR.setPosition(Globals.CLAWROPEN);
        //}
        //sleep(250);

        Trajectory drop = drivetrain.trajectoryBuilder(new Pose2d(15, 6, 0))
                .lineToLinearHeading(new Pose2d(15, 7, 0))
                .build();
        drivetrain.followTrajectory(drop);

        /*
        Trajectory toBD = drivetrain.trajectoryBuilder(new Pose2d(15, 6, 0))
                .addDisplacementMarker(() -> {
                    //Pivot up
                    Globals.pivotScoring();
                    Globals.pivotLeft = true;

                    movePW();
                })
                .lineToLinearHeading(new Pose2d(15, 33, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(toBD);

        //Pivot up


        //Claw Right Release
        //sleep(250);
        while (robot.clawL.getPosition() < Globals.CLAWLOPEN) {
            robot.clawL.setPosition(Globals.CLAWLOPEN);
        }
        //sleep(250);

        //Claw Both Grab
        robot.clawL.setPosition(Globals.CLAWLCLOSED);
        robot.clawR.setPosition(Globals.CLAWRCLOSED);
/*
        Trajectory awayBD = drivetrain.trajectoryBuilder(new Pose2d(15, 33, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(10, 15, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(awayBD);

        Trajectory park = drivetrain.trajectoryBuilder(new Pose2d(10, 15, Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    //Pivot flat
                    Globals.pivotNeutral();

                    movePW();
                })
                .lineToLinearHeading(new Pose2d(0, 10, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(park);

        Trajectory parkpls = drivetrain.trajectoryBuilder(new Pose2d(0, 10, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-10, 5, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(parkpls);
*/
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
