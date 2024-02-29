package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class FarBlue extends LinearOpMode{
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
        Trajectory toStack = drivetrain.trajectoryBuilder(new Pose2d(26, 0, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d (24, -17, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(toStack);
        Trajectory toBD = drivetrain.trajectoryBuilder(new Pose2d(24, -17, Math.toRadians(-90)))
                .lineToLinearHeading( new Pose2d(25, 70, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(toBD);
        //arm up
        Trajectory midAlign = drivetrain.trajectoryBuilder(new Pose2d(25, 70, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(22, 88, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(midAlign);
        Trajectory toStack2 = drivetrain.trajectoryBuilder(new Pose2d(22, 88, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(22, -16, Math.toRadians(-90)))
                .build();
        drivetrain.followTrajectory(toStack2);




    }

}
