package org.firstinspires.ftc.teamcode.pretty.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.pretty.Globals;

public class WristSubsystem {


    //====================
    //scoring variables

    public int wristXOffset = 0;
    public int wristYOffset = 0;

    public double wristXTarget = Globals.WRISTUP;
    public double wristYTarget = Globals.WRISTCENTER;

    //scoring variables
    //====================

    public double xCalculate() {
        double xTarget = (wristXTarget + wristXOffset) / Globals.WXD;
        return xTarget;
    }

    public double yCalculate() {
        double yTarget = (wristYTarget + wristYOffset) / Globals.WYD;
        return yTarget;
    }

    /*
    public void calculateTarget() {
        if (Globals.pivotDown) {
            wristXTarget = Globals.WRISTDOWN - Globals.pivotXDegrees;
            wristYTarget = Globals.WRISTCENTER;
        } else if (Globals.pivotUp) {
            wristXTarget = Globals.WRISTUP - (Globals.PIVOTUP - Globals.pivotXDegrees);
            wristYTarget = Globals.WRISTCENTER - Globals.pivotYDegrees;
        }
    }
    */

    public void set() {
        if (Globals.pivotDown || Globals.pivotUp) {
            //calculateTarget();
            if (Globals.pivotDown) {
                wristXTarget = Globals.WRISTDOWN - Globals.pivotXTarget;
                wristYTarget = Globals.WRISTCENTER;
            } else if (Globals.pivotUp) {
                wristXTarget = Globals.WRISTUP - (Globals.PIVOTUP - Globals.pivotXTarget);
                wristYTarget = Globals.WRISTCENTER - Globals.pivotYTarget;
            }
        } else {
            if (!Globals.wristStored) {
                wristXTarget = Globals.WRISTDOWN;
            } else {
                wristXTarget = Globals.WRISTUP;
            }
            wristYTarget = Globals.WRISTCENTER;
        }
    }



    public void moveOffset(boolean x, boolean negative) { //if x == false then y, if negative == false then positve
        int shift = 3;
        if (negative) {
            shift = -shift;
        }
        if (x) {
            wristXOffset += shift;
        } else {
            wristYOffset += shift;
        }

    }

    public void resetOffset() {
        wristXOffset = 0;
        wristYOffset = 0;
    }

}//end class
