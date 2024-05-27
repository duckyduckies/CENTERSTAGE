package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {


    public static double armIn = 0;

    public static double armOut = 1;

    public static double clawOpen = 0;

    public static double clawClose = 0.5;



    // Camera stuff

    public static int leftBoundary = 940; // left side of detection zone
    public static int rightBoundary = 1010; // right side of detection zone
    public static int middleLine = 735; // detection line y coordinate


    public static int HVLeftBoundary = 930; // left side of detection zone
    public static int HVRightBoundary = 1030; // right side of detection zone
    public static int HVTopBoundary = 685; // top side of detection zone
    public static int HVBottomBoundary = 785; // bottom side of detection zone

    public static int changeThresh = 128;

    public static int signalDetectionMethod = 3; // 1: detect QR code
    // 2: detect vertical 1, 2, 3 lines: require rigid alignment
    // 3: detect H vs V vs Empty: best solution, require less alignment
    // 4: detect H vs V vs Diagonal
    // 5: detect H vs V vs #

}

