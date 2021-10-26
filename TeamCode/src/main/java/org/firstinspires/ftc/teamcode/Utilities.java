package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Subsystems.Wobble;

public class Utilities {


    //wraps number/angle between 180 and -180
    public static double wrapAnglePlusMinus180(final double angle) {
        final double wrapped = angle % 360;

        if (wrapped > 180) {
            return wrapped - 360;
        } else if (wrapped < -180) {
            return wrapped + 360;
        } else {
            return wrapped;
        }
    }
    public static double average2(double a,double b){
        return (a+b)/2;
    }
    public static double average3(double a,double b,double c){
        return (a+b+c)/3;
    }
    public static double average4(double a,double b,double c,double d){
        return (a+b+c+d)/4;
    }

    //tracks robot position
//    public static double odometry(Robot robot) {
//        return (robot.getLeftFrontDrive().getCurrentPosition() + robot.getRightFrontDrive().getCurrentPosition() + robot.getLeftBackDrive().getCurrentPosition() + robot.getRightBackDrive().getCurrentPosition()) / 4;
//    }
//
//    public static double inchToCM(double inchforconvertion) {
//        return inchforconvertion * 2.54;
//    }
//
//    public static void CmPlusPIDDrive(double x, double wantedcm, double wantedgyro, Robot robot, double firstGyrohere) {
//        double lastErrorY = 0;
//        double lastErrorX = 0;
//        double[] xyAxisplus = {0, 0};
//        double lastTick = 0;
//        double errorx = 0;
//        double errory = 0;
//        double[] errorxy = {0, 0};
//        PID pid = new PID(0, 0, 0, 0);
//        double Wantedcm = Math.round(inchToCM(wantedcm) / (Constant.wheelradiusCM * 2 * Math.PI) * 560);
//        while (Wantedcm - (Math.pow((errorx), 2) + Math.pow(errory, 2)) <= 5 && Wantedcm - (Math.pow(errory, 2) + Math.pow(errorx, 2)) >= -5 || x - (Math.pow(errory, 2) + Math.pow(errorx, 2)) < 5 && (x - (Math.pow(errory, 2) + Math.pow(errorx, 2)) > -5)) {
//            pid.setWanted(wantedgyro);
//            double robotOdometry = odometry(robot);
//            xyAxisplus = new double[]{Math.sin(robot.dt.getAngle(robot.getImu(), firstGyrohere) - wantedgyro) * robotOdometry, Math.cos(robot.dt.getAngle(robot.getImu(), firstGyrohere) - wantedgyro) * robotOdometry};//0=y 1=x
//            errorx += xyAxisplus[1] - lastErrorX;
//            errory += xyAxisplus[0] - lastErrorY;//1=x 0=y
//            robot.dt.calculatePower(Math.max(errorx, 0.3), Math.max(errory, 0.3), pid.update(robotOdometry), 0.5);
//            lastErrorY = errory;
//            lastErrorX = errorx;
//            lastTick = xyAxisplus[0];
//            //pid.update(Odomatry[0]); useless cuz you put pid uptade in cac power
//        }
//    }



}