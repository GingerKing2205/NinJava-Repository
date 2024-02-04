package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autopathing.InduCenter;
import org.firstinspires.ftc.teamcode.autopathing.InduCenterRed;
import org.firstinspires.ftc.teamcode.autopathing.InduLeft;
import org.firstinspires.ftc.teamcode.autopathing.InduLeftRed;
import org.firstinspires.ftc.teamcode.autopathing.InduRight;
import org.firstinspires.ftc.teamcode.autopathing.InduRightRed;
import org.firstinspires.ftc.teamcode.huskyLens.HuskyFunc;

@Autonomous(name = "RedAuto", preselectTeleOp = "GinjaKarpTelePID")
public class RedAuto extends LinearOpMode {
    private HuskyFunc cam = new HuskyFunc();

    private InduCenterRed Center = new InduCenterRed();

    private InduLeftRed Left = new InduLeftRed();

    private InduRightRed Right = new InduRightRed();

    private int border = 150;

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
