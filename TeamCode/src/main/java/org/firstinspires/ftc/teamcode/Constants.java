package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    public static double wristUp = 1;

    public static double wristDown = 0;

    public static double clawOpen = 0;

    public static double clawClose = 1;


    // Camera stuff

    public static int leftBoundary = 940; // left side of detection zone
    public static int rightBoundary = 1010; // right side of detection zone
    public static int middleLine = 735; // detection line y coordinate


    public static int HVLeftBoundary = 930; // left side of detection zone
    public static int HVRightBoundary = 1030; // right side of detection zone
    public static int HVTopBoundary = 685; // top side of detection zone
    public static int HVBottomBoundary = 785; // bottom side of detection zone

    public static int changeThresh = 128;

    public static int signalDetectionMethod = 1; // 1: Max mean method
    // 2: detect histogram


}

