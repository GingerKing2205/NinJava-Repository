package org.firstinspires.ftc.teamcode.pretty;

import org.apache.commons.math3.util.PivotingStrategyInterface;

public class Globals {


    //====================
    //wrist constants

    public static final double WXD = 150;
    public static final double WYD = 50;

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

    public static final double SIDECENTER = 0;
    public static final double SIDELEFT = -25;
    public static final double SIDERIGHT = 13;

    public static final double SL = 18; //inches
    public static final double PHEIGHT = 7.5; //inches was 4

    //pivot constants
    //====================


    //====================
    //slide constants

    public static final double SI = -220;// -3300/15 ticks / inches - was 2462/17

    public static final double SLIDEXTEND = -3300;

    public static final double SLIDERETRACT = 0;

    //slide constants
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

    public static int pivotXOffset = 0;
    public static int pivotYOffset = 0;

    public static double pivotXTarget = 0;
    public static double pivotYTarget = 0;

    public static double pivotXDegrees = 0;
    public static double pivotYDegrees = 0;

    //pivot globals
    //====================


    //====================
    //slide globals

    public static double slideInches = 0;

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
    }

    public static void pivotNeutral() {
        pivotUp = false;
        pivotDown = false;
        pivotRight = false;
        pivotLeft = false;
        wristStored = false;
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
    }

    public static void readPositions(int pX, int pY, int sP) {
        pivotXDegrees = pX / PXD;
        pivotYDegrees = pY / PYD;
        slideInches = sP / SI;
    }

    //set variables
    //====================

}
