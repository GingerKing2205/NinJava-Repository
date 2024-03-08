package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class FarLeftBlue extends LinearOpMode{
    @Override
    public void runOpMode(){
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Trajectory lineTo = drivetrain.trajectoryBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(20, 0, 0))
                .build();
        drivetrain.followTrajectory(lineTo);

        Trajectory lineTo2 = drivetrain.trajectoryBuilder(new Pose2d(20,0,0))
                .lineToLinearHeading(new Pose2d (22, 0, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(lineTo2);
        Trajectory toStack = drivetrain.trajectoryBuilder(new Pose2d(22, 0, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d (24, -17, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(toStack);
        Trajectory trussAlign = drivetrain.trajectoryBuilder(new Pose2d(24, -15, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(17, -8, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(trussAlign);
        Trajectory throughTruss = drivetrain.trajectoryBuilder(new Pose2d(15, -8, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(17, 60, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(throughTruss);
        Trajectory toBD = drivetrain.trajectoryBuilder(new Pose2d(17, 60, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(30, 85, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(toBD);

        telemetry.addData("hehe", drivetrain.getPoseEstimate());



    }

}
