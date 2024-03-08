
package org.firstinspires.ftc.teamcode.cameratest.huskyLens;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;


import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;


public class HuskyFunc {


    private final int READ_PERIOD = 1;

    public HuskyLens huskyLens;

    public void init(HardwareMap hardwareMap) {

        huskyLens = hardwareMap.get(HuskyLens.class, "husky");

        huskyLens.initialize();

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

    }//ends init


    public int scan() {

        List<HuskyLens.Block> objectList;

        HuskyLens.Block prop;

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        rateLimit.expire();

        boolean scanned = false;

        int objectX = 0;

        int loopTimer = 0;

        while(!scanned && loopTimer <= 500) {
            loopTimer++;
            if (!rateLimit.hasExpired()) {

                continue;

            }

            rateLimit.reset();


            objectList = Arrays.asList(huskyLens.blocks());

            for (HuskyLens.Block TeamProp : objectList) {

                prop = TeamProp;

                objectX = prop.x;


                if (prop.id == 1) {

                    scanned = true;

                    objectX = prop.x;


                }//end ifLoop
            }//end forLoop
        }//end whileLoop

        //return objectX;

        if (scanned) {
            return objectX;
        } else {
            return -1;
        }

    }//end runOpMode
}//end class