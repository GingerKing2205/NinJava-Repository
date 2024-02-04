package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autopathing.InduCenter;
import org.firstinspires.ftc.teamcode.autopathing.InduLeft;
import org.firstinspires.ftc.teamcode.autopathing.InduRight;
import org.firstinspires.ftc.teamcode.huskyLens.HuskyFunc;
@Autonomous(name = "BlueAuto", preselectTeleOp = "GinjaKarpTelePID")
public class BlueAuto extends LinearOpMode {
    private HuskyFunc cam = new HuskyFunc();

    private InduCenter Center = new InduCenter();

    private InduLeft Left = new InduLeft();

    private InduRight Right = new InduRight();

    private int border = 200;

    @Override
    public void runOpMode() throws InterruptedException {

        cam.init(hardwareMap);

        telemetry.addData("Press Start", "now" );

        waitForStart();

        if(opModeIsActive()) {

            int xLoc = cam.scan();

            if (xLoc <= border && xLoc > 0) {

                //Center auto
                telemetry.addData("Center Auto", xLoc);
                Center.runOpMode(hardwareMap, telemetry);

            } else if (xLoc >= border ) {

                //Right auto
                telemetry.addData("Right Auto", xLoc);
                Right.runOpMode(hardwareMap, telemetry);

            } else {

                //Left auto
                telemetry.addData("Left Auto", xLoc);
                Left.runOpMode(hardwareMap,telemetry);

            }// end if2
        }//end if1



    }//ends scan
}//ends class
