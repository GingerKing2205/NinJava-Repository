package org.firstinspires.ftc.teamcode.pretty.subsystems;

import org.firstinspires.ftc.teamcode.pretty.Globals;

public class PivotSubsystem {





    public static double xCalculate() {
        double xTarget = (Globals.pivotXTarget + Globals.pivotXOffset) * Globals.PXD;
        return xTarget;
    }

    public static double yCalculate() {
        double yTarget = (Globals.pivotYTarget + Globals.pivotYOffset) * Globals.PYD;
        return yTarget;
    }

    /*
    public static void calculateTarget() {
        if (Globals.pivotDown) {
            Globals.pivotXTarget = Math.toDegrees(-Math.asin(Globals.PHEIGHT / (Globals.SL + Globals.slideInches * Globals.SI)));
            Globals.pivotYTarget = Globals.SIDECENTER;
        } else if (Globals.pivotUp) {
            Globals.pivotXTarget = Globals.PIVOTUP;
            if (Globals.pivotRight) {
                Globals.pivotYTarget = Globals.SIDERIGHT;
            } else if (Globals.pivotLeft) {
                Globals.pivotYTarget = Globals.SIDELEFT;
            } else {
                Globals.pivotYTarget = Globals.SIDECENTER;
            }
        }
    }
     */

    public static void set() {
        if (Globals.pivotDown || Globals.pivotUp) {
            //calculateTarget();
            if (Globals.pivotDown) {
                if (Globals.pivotStack) {
                    Globals.pivotXTarget = Math.toDegrees(-Math.asin((Globals.PHEIGHT - Globals.stackHeight) / (Globals.SL + Globals.slideInches)));
                    Globals.pivotYTarget = Globals.SIDECENTER;
                } else {
                    Globals.pivotXTarget = Math.toDegrees(-Math.asin(Globals.PHEIGHT / (Globals.SL + Globals.slideInches)));
                    Globals.pivotYTarget = Globals.SIDECENTER;
                }
            } else if (Globals.pivotUp) {
                Globals.pivotXTarget = Globals.PIVOTUP;
                if (Globals.pivotRight) {
                    Globals.pivotYTarget = Globals.SIDERIGHT;
                } else if (Globals.pivotLeft) {
                    Globals.pivotYTarget = Globals.SIDELEFT;
                } else {
                    Globals.pivotYTarget = Globals.SIDECENTER;
                }
            }
        } else {
            if (Globals.wristStored) {
                Globals.pivotXTarget = Globals.PIVOTSTORED;
            } else {
                Globals.pivotXTarget = Globals.PIVOTFLAT;
            }
            Globals.pivotYTarget = Globals.SIDECENTER;
        }
    }



    public static void moveOffset(boolean x, boolean negative) { //if x == false then y, if negative == false then positve
        int shift = 3;
        if (negative) {
            shift = -shift;
        }
        if (x) {
            Globals.pivotXOffset += shift;
        } else {
            Globals.pivotYOffset += shift;
        }

    }

    public static void resetOffset() {
        Globals.pivotRight = false;
        Globals.pivotLeft = false;
        Globals.pivotXOffset = 0;
        Globals.pivotYOffset = 0;
    }

}
