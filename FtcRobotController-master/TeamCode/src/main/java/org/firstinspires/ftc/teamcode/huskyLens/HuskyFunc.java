
package org.firstinspires.ftc.teamcode.huskyLens;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Disabled.BlueAuto;


import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;


public class HuskyFunc extends BlueAuto {


    private final int READ_PERIOD = 1;

    public HuskyLens huskyLens;

    public void init(HardwareMap hardwareMap) {

        huskyLens = hardwareMap.get(HuskyLens.class, "husky");

        huskyLens.initialize();
    }//ends init


    public int scan() {

        List<HuskyLens.Block> objectList;

        HuskyLens.Block prop;

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        rateLimit.expire();

        boolean scanned = false;

        int objectX = 0;

        int loopCounter = 1;

        while(!scanned && loopCounter <= 20) {
            loopCounter++;
            if (!rateLimit.hasExpired()) {

                continue;

            }

            rateLimit.reset();


            objectList = Arrays.asList(huskyLens.blocks());

            for (HuskyLens.Block TeamProp : objectList) {

                prop = TeamProp;


                if (prop.id == 1) {

                    scanned = true;

                    objectX = prop.x;


                }//end ifLoop
            }//end forLoop
        }//end whileLoop

        return objectX;

    }//end runOpMode
}//end class