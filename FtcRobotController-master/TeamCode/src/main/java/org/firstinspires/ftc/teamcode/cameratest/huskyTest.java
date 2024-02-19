
package org.firstinspires.ftc.teamcode.cameratest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;


import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 * 
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "huskyTest", group = "Test")
public class huskyTest extends LinearOpMode {

    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;




    @Override
    public void runOpMode()
    {
        List<HuskyLens.Block> objectList;

        HuskyLens.Block prop;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        huskyLens = hardwareMap.get(HuskyLens.class, "husky");

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            //reads whether it scans
           /* HuskyLens.Block[] blocks = huskyLens.blocks();
           telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            } */
            //reads whether is scans
            objectList = Arrays.asList(huskyLens.blocks());

            for (HuskyLens.Block TeamProp : objectList) {
                prop = TeamProp;
                telemetry.addData("Block ID", prop.id);
                telemetry.addData("Block X", prop.x);
                telemetry.addData("Block Y", prop.y);
                telemetry.update();
                if (prop.id == 1) {
                    if (prop.x <= 150) {
                        //Left auto
                        telemetry.addData("Left Auto", prop.x);
                    } else if (prop.x <= 350) {
                        //Center auto
                        telemetry.addData("Center Auto", prop.x);
                    } else {
                        //Right auto
                        telemetry.addData("Right Auto", prop.x);
                    }
                }
            }



        }
    }
}