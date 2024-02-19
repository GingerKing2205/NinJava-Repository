package org.firstinspires.ftc.teamcode.OldOp;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


//@Config
public class liftPID {
    public PIDController controller;

    public static double p = 0.0023, i = 0, d = 0.0005;
    public static double f=0.12;//was .1

    public int target = 0;

    public final double ticks_in_degree = 5264.0 / 180;

    public DcMotorEx leftFlip;

    public void init(HardwareMap hardwareMap) {

        leftFlip = hardwareMap.get(DcMotorEx.class, "leftFlip");


        controller = new PIDController(p,i,d);



    }//ends init

    public void loop() {

        controller.setPID(p, i, d);
        int armPos = leftFlip.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;


        double power = pid - ff;

        leftFlip.setPower(power);


    }//ends loop

    public void kill() {
        leftFlip.setPower(0);
    }

    public void reset(){
        leftFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFlip.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }//ends reset

}
