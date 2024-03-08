package org.firstinspires.ftc.teamcode.drive.opmode.autos.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.opmode.autos.blue.close.*;
import org.firstinspires.ftc.teamcode.cameratest.huskyLens.HuskyFunc;
import org.firstinspires.ftc.teamcode.pretty.hardware;

@Autonomous (name = "CloseBlueAuto", group = "autos", preselectTeleOp = "StateTele")
public class CloseBlueAuto extends LinearOpMode {

    public HuskyFunc husky = new HuskyFunc();
    public hardware robot = new hardware();

    public CloseLeftBlue left = new CloseLeftBlue();
    public CloseRightBlue right = new CloseRightBlue();
    public CloseCenterBlue center = new CloseCenterBlue();

    @Override
    public void runOpMode() {

        husky.init(hardwareMap);

        robot.init(hardwareMap, false);
        robot.slide.setPower(0.25);

        waitForStart();

        int objectX = husky.scan();
        telemetry.addData("husky", objectX);
        telemetry.update();

        sleep(1000);

        if (objectX > 200) {
            right.run(hardwareMap);
        } else if (objectX <= 200 && objectX >= 0) {
            center.run(hardwareMap);
        } else {
            left.run(hardwareMap);
        }

    }

}
