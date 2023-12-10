package org.firstinspires.ftc.teamcode.Disabled;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Disabled.pidClassTest;

@TeleOp
@Disabled
public class armClassTest extends LinearOpMode {

    private pidClassTest arm = new pidClassTest();

    //public DcMotorEx leftFlip;

    @Override

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //leftFlip = hardwareMap.get(DcMotorEx.class, "leftFlip");

        arm.init(hardwareMap, (MultipleTelemetry) telemetry);



        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()) {
            arm.loop((MultipleTelemetry) telemetry);


        }//ends opMode
    }//ends runOpMode
}//ends class

