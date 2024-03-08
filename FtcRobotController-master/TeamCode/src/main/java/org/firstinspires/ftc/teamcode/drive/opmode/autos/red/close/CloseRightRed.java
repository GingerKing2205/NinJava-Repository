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
public class CloseRightRed/* extends LinearOpMode*/ {
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


        //Unfold
        Globals.pivotNeutral();

        movePW();

        Trajectory toPurpleSpike = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(24.5, -9, 0))
                .build();
        drivetrain.followTrajectory(toPurpleSpike);

        //Left Claw Release
        //sleep(250);
        //while (robot.clawL.getPosition() < Globals.CLAWLOPEN) {
            robot.clawL.setPosition(Globals.CLAWLOPEN);
        //}
        //sleep(250);

        Trajectory awayPurpleSpike = drivetrain.trajectoryBuilder(new Pose2d(24.5,-9,0))
                .lineToLinearHeading(new Pose2d(20, -9, 0))
                .build();
        drivetrain.followTrajectory(awayPurpleSpike);

        Trajectory toBD = drivetrain.trajectoryBuilder(new Pose2d(20, -9,  0))
                .addDisplacementMarker(() -> {
                    //Pivot up
                    Globals.pivotScoring();
                    Globals.pivotLeft = true;

                    movePW();
                })
                .lineToLinearHeading(new Pose2d(23, -34, Math.toRadians(90)))
                .build();
        drivetrain.followTrajectory(toBD);


        //Claw Right Release
        //sleep(250);
        //while (robot.clawR.getPosition() > Globals.CLAWROPEN) {
            robot.clawR.setPosition(Globals.CLAWROPEN);
        //}
        //sleep(250);

        Trajectory awayBD = drivetrain.trajectoryBuilder(new Pose2d(23, -34, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(10, -19, Math.toRadians(90)))
                .build();
        drivetrain.followTrajectory(awayBD);

        Trajectory park = drivetrain.trajectoryBuilder(new Pose2d(10, -19, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    //Pivot flat
                    Globals.pivotNeutral();
                    robot.clawR.setPosition(Globals.CLAWRCLOSED);
                    robot.clawL.setPosition(Globals.CLAWLCLOSED);

                    movePW();
                })
                .lineToLinearHeading(new Pose2d(8, -44, Math.toRadians(90)))
                .build();
        drivetrain.followTrajectory(park);

        Trajectory parkpls = drivetrain.trajectoryBuilder(new Pose2d(8, -44, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(8, -45, Math.toRadians(90)))
                .build();
        drivetrain.followTrajectory(parkpls);




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
