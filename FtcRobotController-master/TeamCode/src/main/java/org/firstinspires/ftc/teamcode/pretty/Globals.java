package org.firstinspires.ftc.teamcode.pretty;

import org.apache.commons.math3.util.PivotingStrategyInterface;

public class Globals {


    //====================
    //auto constants

    public static final double PIXEL = 0.5; //inches

    //auto constants
    //====================


    //====================
    //claw constants

    public static final double CLAWLCLOSED = 0; //inches
    public static final double CLAWLOPEN = 270; //inches

    public static final double CLAWRCLOSED = 270; //inches
    public static final double CLAWROPEN = 0; //inches

    //claw constants
    //====================


    //====================
    //wrist constants

    public static final double WXD = 150; //degrees
    public static final double WYD = 50; //degrees

    public static final double WRISTDOWN = 0; //degrees
    public static final double WRISTUP = 150; //degrees
    public static final double WRISTCENTER = 25; //degrees

    //wrist constants
    //====================


    //====================
    //pivot constants

    public static final double PXD = 29.84; //Ticks To Degrees  TTI = Ticks To Inches
    public static final double PYD = PXD;// is 29.84

    public static final double PIVOTFLAT = 0; //degrees
    public static final double PIVOTUP = 110; //degrees
    public static final double PIVOTSTORED = -10; //degrees

    public static final double SIDECENTER = 0; //degrees
    public static final double SIDELEFT = -25; //degrees
    public static final double SIDERIGHT = 13; //degrees

    public static final double SL = 18; //inches
    public static final double PHEIGHT = 6; //inches was 7.5

    //pivot constants
    //====================


    //====================
    //slide constants

    public static final double SI = -226.66;// 3400/15 ticks / inches - was -3300/15

    public static final double SLIDEXTEND = -3400; //ticks was -3300

    public static final double SLIDERETRACT = -220; //ticks was 0

    //slide constants
    //====================


    //====================
    //auto globals

    public static boolean pivotStack = false;
    public static double stackHeight = 4 * PIXEL; //inches

    //auto globals
    //====================


    //====================
    //wrist globals

    public static boolean wristStored = false;

    //wrist globals
    //====================


    //====================
    //pivot globals

    public static boolean pivotUp = false;
    public static boolean pivotDown = false;

    public static boolean pivotLeft = false;
    public static boolean pivotRight = false;

    public static int pivotXOffset = 0; //degrees
    public static int pivotYOffset = 0; //degrees

    public static double pivotXTarget = 0; //degrees
    public static double pivotYTarget = 0; //degrees

    public static double pivotXDegrees = 0; //degrees
    public static double pivotYDegrees = 0; //degrees

    //pivot globals
    //====================


    //====================
    //slide globals

    public static double slideInches = 0; //inches
    public static double slideTarget = 0; //inches

    //slide globals
    //====================


    //====================
    //set variables

    public static void pivotScoring() {
        pivotUp = true;
        pivotDown = false;
        pivotRight = false;
        pivotLeft = false;
        wristStored = false;
        stackHeight = 4;
    }

    public static void pivotNeutral() {
        pivotUp = false;
        pivotDown = false;
        pivotRight = false;
        pivotLeft = false;
        wristStored = false;
        pivotStack = false;
        stackHeight = 4;
    }

    public static void pivotstored() {
        pivotUp = false;
        pivotDown = false;
        pivotRight = false;
        pivotLeft = false;
        wristStored = true;
    }

    public static void pivotIntaking() {
        pivotUp = false;
        pivotDown = true;
        pivotRight = false;
        pivotLeft = false;
        wristStored = false;
        stackHeight = 4;
    }

    public static void readPositions(int pX, int pY, int sP) {
        pivotXDegrees = pX / PXD;
        pivotYDegrees = pY / PYD;
        slideInches = sP / SI;
    }

    public static void stackUp() {
        stackHeight -= PIXEL;
        pivotStack = true;
        if (stackHeight < 1) {
            stackHeight = 4;
            pivotStack = false;
        }
    }

    //set variables
    //====================

}
