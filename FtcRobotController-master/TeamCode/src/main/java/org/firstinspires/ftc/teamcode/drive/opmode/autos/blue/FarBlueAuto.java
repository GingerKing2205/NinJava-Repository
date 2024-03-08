package org.firstinspires.ftc.teamcode.drive.opmode.autos.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.cameratest.huskyLens.HuskyFunc;
import org.firstinspires.ftc.teamcode.pretty.hardware;
import org.firstinspires.ftc.teamcode.drive.opmode.autos.blue.far.*;

@Autonomous (name = "FarBlueAuto", group = "autos", preselectTeleOp = "StateTele")
public class FarBlueAuto extends LinearOpMode {

    public HuskyFunc husky = new HuskyFunc();
    public hardware robot = new hardware();

    public FarLeftBlue left = new FarLeftBlue();
    public FarRightBlue right = new FarRightBlue();
    public FarCenterBlue center = new FarCenterBlue();

    @Override
    public void runOpMode() {

        husky.init(hardwareMap);

        robot.init(hardwareMap, false);
        robot.slide.setPower(0.25);

        waitForStart();

        int objectX = husky.scan();
        telemetry.addData("husky", objectX);
        telemetry.update();

        //sleep(5000);

        if (objectX > 200) {
            right.run(hardwareMap);
            telemetry.addData("auto", "right");
        } else if (objectX <= 200 && objectX >= 0) {
            center.run(hardwareMap);
            telemetry.addData("auto", "center");
        } else {
            left.run(hardwareMap);
            telemetry.addData("auto", "left");
        }
        telemetry.update();

        sleep(2000);

    }

}
