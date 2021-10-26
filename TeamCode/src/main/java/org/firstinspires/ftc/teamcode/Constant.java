package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constant {
    public static double wheelScopeInCm = 55;//old 52.2
    public static double WheelScopeInCmItamer = 45;

    public static double wheelScopeInTicks = 560;
    public static double transferServoClosed = 0.5;
    public static double transferServoOpen = 0.15;//0.15


    public static int min = 1;
    public static int max = 0;


    public static double clawPositionClosed = 0.85;
    public static double clawPositionOpened = 0.5;

    public static double transferringPower = 0.6;
    public static double transferDepletingPower = -1;

    public static double intakeCollectingPower = 0.8;
    public static double intakeDepletionPower = -0.7;

    public static double vexHoldingPower = -0.2;
    public static double ac_and_dc_range = 13440;
    public static double gyroKP = -0.0277;
    public static double gyroKI = 0;
    public static double gyroKD = 0;
    public static double gyroKF = 0;
    public static double gyroIZone = 0;
    public static double wanted = 90;

    public static int midgoallong = 1700;
    public static int longhighgoal = 1800;


    public static int highGoalTPS = 1850; //old 1800
    public static int middleGoalTPS = 1450;
    public static int powerShotTPS = 1625;// megiddolions/FTC-18833-2021
    public static double xkp = 0.2;
    public static int lowGoalTPS = 1250;
    public static int tpsError = 125;
    public static double pfortransfer = 0.0040;
    public static double shooterKP = 0.0005;
    public static double shooterKI = 0;
    public static double shooterKD = 0.2;
    public static double shooterKF = 0.00063;
    public static double shooterIZone = 0;
    public static double speedFactor = 0.85;

    public static double driveKp = 0.027;
    public static double driveKi = 0;
    public static double driveKd = 0.25;
    public static double driveKf = 0;
    public static double driveIZone = 0;

    public static double firstTurn = 0;
    public static double secondTurn = 0;
    public static int waitBetweenShots = 0;

}