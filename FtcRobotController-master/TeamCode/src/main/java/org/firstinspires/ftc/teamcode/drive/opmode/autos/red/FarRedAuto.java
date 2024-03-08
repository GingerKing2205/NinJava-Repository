package org.firstinspires.ftc.teamcode.drive.opmode.autos.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.cameratest.huskyLens.HuskyFunc;
import org.firstinspires.ftc.teamcode.drive.opmode.autos.red.far.*;
import org.firstinspires.ftc.teamcode.pretty.hardware;

@Autonomous (name = "FarRedAuto", group = "autos", preselectTeleOp = "StateTele")
public class FarRedAuto extends LinearOpMode {

    public HuskyFunc husky = new HuskyFunc();
    public hardware robot = new hardware();

    public FarLeftRed left = new FarLeftRed();
    public FarRightRed right = new FarRightRed();
    public FarCenterRed center = new FarCenterRed();

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
