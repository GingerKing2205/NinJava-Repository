package org.firstinspires.ftc.teamcode.drive.opmode.autos.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.cameratest.huskyLens.HuskyFunc;
import org.firstinspires.ftc.teamcode.drive.opmode.autos.red.close.*;
import org.firstinspires.ftc.teamcode.pretty.hardware;

import java.util.jar.Attributes;

@Autonomous (name = "CloseRedAuto", group = "autos", preselectTeleOp = "StateTele")
public class CloseRedAuto extends LinearOpMode {

    public HuskyFunc husky = new HuskyFunc();
    public hardware robot = new hardware();

    public CloseLeftRed left = new CloseLeftRed();
    public CloseRightRed right = new CloseRightRed();
    public CloseCenterRed center = new CloseCenterRed();

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
        } else if (objectX <= 200 && objectX > 0) {
            center.run(hardwareMap);
        } else {
            left.run(hardwareMap);
        }

    }

}
