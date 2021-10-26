package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.PID;

import static org.firstinspires.ftc.teamcode.Utilities.average2;
import static org.firstinspires.ftc.teamcode.Utilities.wrapAnglePlusMinus180;


public class DriveTrain {

    private DcMotor lf, rf, lb, rb;
    private BNO055IMU imu;
    private double odometry;

    private PID pidGyro = new PID(Constant.driveKp, Constant.driveKi, Constant.driveKd, Constant.driveKf, Constant.driveIZone);
    private double[] odomatry = {0, 0, 0, 0};

    //returns imu
    public BNO055IMU getImu() {
        return imu;
    }

    //returns all drivetrain motor powers
    public double[] getPowers() {
        return new double[]{lf.getPower(), rf.getPower(), lb.getPower(), rb.getPower()};
    }

    //returns encoder position of all drivetrain motors
    public double[] getPositions() {
        double[] positions = {lf.getCurrentPosition(), rf.getCurrentPosition(), lb.getCurrentPosition(), rb.getCurrentPosition()};
        return positions;
    }

    //drives mecanum drivetrain according to drivers position and not according to robots position
    public void fieldCentric(double x, double y, double r, double speedFactor, double firstGyro) {
        double length = Math.hypot(x, y);
        double robotAngle = getAngle(firstGyro);
        double joystickAngle =((Math.toDegrees(Math.atan2(y, x)) + 360) % 360);
        double finalAngle = (robotAngle + joystickAngle + 360) % 360;
        double completeY = Math.sin(Math.toRadians(finalAngle)) * length;
        double completeX = Math.cos(Math.toRadians(finalAngle)) * length;

        calculatePower(completeX, completeY, r, speedFactor);
    }

    //returns you current angle
    public double getAngle(double firstGyro) {//gives you gyro angle
        return wrapAnglePlusMinus180(wrapAnglePlusMinus180(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) - firstGyro);
    }

    //calculates power for the motors according to a vector
    private void calculatePower(double x, double y, double r, double speedFactor) {
        double lfPower = y - x - r;
        double lbPower = y + x - r;
        double rfPower = y - x + r;
        double rbPower = y + x + r;
        double max = Math.max(Math.max(Math.max(Math.max(Math.abs(rfPower), Math.abs(lfPower)), Math.abs(rbPower)), Math.abs(lbPower)), 1);
        lfPower /= max;
        lbPower /= max;
        rfPower /= max;
        rbPower /= max;
        lfPower *= speedFactor;
        lbPower *= speedFactor;
        rfPower *= speedFactor;
        rbPower *= speedFactor;
        setMotorPower(lfPower, lbPower, rbPower, rfPower, 0);
    }

    //gives power to the motors
    private void setMotorPower(double lfPower, double lbPower, double rfPower, double rbPower, double steering) {
        lf.setPower(lfPower + steering);
        lb.setPower(lbPower + steering);
        rb.setPower(rbPower - steering);
        rf.setPower(rfPower - steering);
    }

    //drivetrain initiation
    public void init(HardwareMap hardwareMap) {
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
    }

    //turns according to degrees
    public void gyroTurn(double wanted, double firstGyro,Telemetry telemetry,TelemetryPacket packet,FtcDashboard dash) {
        PID gyroPID = new PID(Constant.gyroKP, Constant.gyroKI, Constant.gyroKD, Constant.gyroKF, Constant.gyroIZone);
        gyroPID.setWanted(wanted);

        double angle = wrapAnglePlusMinus180(getAngle(firstGyro));
        double error = wrapAnglePlusMinus180(wanted - angle);
        double speed = 0;
        gyroPID.kP = 0;
        while (Math.abs(error) > 0.5) {
            angle = getAngle(firstGyro);
            speed = gyroPID.update(angle);
            if(Math.abs(error)<15){
                speed = speed / (Math.abs(speed/0.35));
            }
            calculatePower(0, 0, speed, 0.5);
            gyroPID.kP = Constant.gyroKP;
            gyroPID.kI = Constant.gyroKI;
            gyroPID.kD = Constant.gyroKD;
            gyroPID.kF = Constant.gyroKF;
            gyroPID.iZone = Constant.gyroIZone;

            double[] powers = getPowers();
            error = wrapAnglePlusMinus180(wanted - angle);
            telemetry.addData("getAngle", angle);
            telemetry.addData("error", error);
            telemetry.addData("speed", speed);
            packet.put("error", error);
            packet.put("speed", speed);
            packet.put("angle", angle);
            packet.put("wanted", wanted);
            packet.put("lf",powers[0]);
            packet.put("rf",powers[1]);
            packet.put("lb",powers[1]);
            packet.put("rb",powers[2]);
            telemetry.update();
            dash.sendTelemetryPacket(packet);
        }
        calculatePower(0, 0, 0, 0);
    }

    public void CmDrivePid(double wanted, double speed,double wantedGyro, double firstGyro, FtcDashboard dash, TelemetryPacket packet) {
        double wheelscopeincm =Constant.WheelScopeInCmItamer;
        double wantedTicks = Constant.wheelScopeInTicks * (wanted / wheelscopeincm);
        double[] firstTicks = {lf.getCurrentPosition(), rf.getCurrentPosition(), lb.getCurrentPosition(), rb.getCurrentPosition()};

        PID pidGyro = new PID(Constant.driveKp, Constant.driveKi, Constant.driveKd, Constant.driveKf, Constant.driveIZone);
        pidGyro.setWanted(wantedGyro);
        while (true) {
            double currentAngle = getAngle(firstGyro);
            double currentLeft =  lb.getCurrentPosition()-firstTicks[2];
            double currentRight = average2(rf.getCurrentPosition() - firstTicks[1], rb.getCurrentPosition() - firstTicks[3]);
            if (Math.abs(wantedTicks - Math.abs(average2(currentLeft, currentRight))) <= 10)break;
            packet.put("angle", currentAngle);
            packet.put("averageLeft", currentLeft);
            packet.put("averageRight", currentRight);
            dash.sendTelemetryPacket(packet);
            setMotorPower(speed, speed, speed, speed, pidGyro.update(currentAngle));
        }
        setMotorPower(0, 0, 0, 0, 0);
    }

    public double whatSpeed(double current, double wanted, double minspeed, double maxspeed, int minusorplus) {
        double speed = 0;
        if (wanted != 0)
            speed = (((Math.abs(current) / Math.abs(wanted)) * (maxspeed - minspeed) + (minspeed)) * minusorplus);
        else
            speed = 0;
        return speed;
    }


    public double[] Odometry(double Lf, double Rf, double Lb, double Rb) {
        double [] motorY = {(long) ((rb.getCurrentPosition() - Rb) * 0.707), (long) (0.707 * (lb.getCurrentPosition() - Lb)), (long) (0.707 * (rf.getCurrentPosition() - Rf)), (long) (0.707 * (rb.getCurrentPosition() - Rb))};
        double [] motorX = {(motorY[0] * -1), motorY[1] * -1, motorY[2], motorY[3]};
        double averageY = (long) ((motorY[0] + motorY[1] + motorY[2] + motorY[3]) / 4);
        double averageX = (long) (motorX[0] + motorX[1] + motorX[2] + motorX[3] / 4);
        double distance = (long) Math.sqrt(Math.pow(averageX, 2) + Math.pow(averageY, 2));
        double[] location = {averageX, averageY, distance};
        return location;
    }

    public double[] speedcs(double current,double ac, int minusorplus, double dec, double x, double wantedTicks, double decx, double acx,double minspeed,double maxspeed) {
        double speed = 0;
        double distance2 = current;
        double lastspeed=0;
        double speedx=0;
        if (Math.abs(distance2) <= Math.abs(ac) && Math.abs(distance2) < Math.abs(dec) || (Math.abs(distance2) <= acx) && (Math.abs(distance2) <= decx)) {
            speed = whatSpeed(distance2, ac, minspeed, maxspeed, minusorplus) - ((whatSpeed(distance2, ac, 0.5, 0.8, minusorplus) - lastspeed)*0.015);
            speedx = whatSpeed(distance2, acx, minspeed, maxspeed, minusorplus);
        } else if (Math.abs(distance2) >= Math.abs(dec) || Math.abs(distance2) >= decx) {
            speed = whatSpeed(Math.abs(wantedTicks) - Math.abs(distance2), ac, 0.3, 0.6, minusorplus) - ((whatSpeed(Math.abs(wantedTicks) - Math.abs(distance2), ac, 0.3, 0.6, minusorplus) - lastspeed)*0.015);
            speedx = whatSpeed(Math.abs(wantedTicks) - Math.abs(distance2), acx, 0.3, 0.6, minusorplus);
        } else if (x != 0)
            speed = 0.8 * minusorplus;
        else
            speedx = 0.8 * minusorplus;
        double[] speeds = {speed,speedx};
        lastspeed = speed;
        return speeds;
    }




    public void CmDrivePid2(double x, double wantedgyro, double wanted, double firstGyro,double minspeed,double maxspeed, FtcDashboard dash, TelemetryPacket packet, double ac_and_dec) {
        double wantedTickss = (Constant.wheelScopeInTicks * (wanted / Constant.wheelScopeInCm)) - 10;
        int minusorplus = (int) Math.signum(wantedTickss);
        double wantedTicks = Math.abs(wantedTickss);
        double ac = wantedTicks/100 * ac_and_dec;
        double dec = wantedTicks/100 * (100-ac_and_dec);
        double acx = x/100 * ac_and_dec;
        boolean exit = true;
        double decx = x/100 * (100-ac_and_dec);
        double[] firstTicks = {rf.getCurrentPosition(), lb.getCurrentPosition(), rb.getCurrentPosition(),lf.getCurrentPosition()};
        PID pidGyro = new PID(Constant.driveKp, Constant.driveKi, Constant.driveKd, Constant.driveKf, Constant.driveIZone);
        pidGyro.setWanted(wantedgyro);
        while (exit == true) {
            double[] distance = Odometry(firstTicks[3],firstTicks[0],firstTicks[1],firstTicks[2]);
            double averageright = average2(rf.getCurrentPosition()-firstTicks[0], rb.getCurrentPosition()-firstTicks[2]);
            double averageleft = average2(lb.getCurrentPosition()-firstTicks[1], lf.getCurrentPosition()-firstTicks[3]);
            double odometry =distance[1];
            double currentAngle = getAngle(firstGyro);
            double[] speeds = speedcs(odometry,ac,minusorplus,dec,x,wantedTicks,decx,acx,minspeed,maxspeed);
            if (((Math.abs(wantedTicks) - Math.abs(odometry) <= 50))) {
                lf.setPower(0);
                rb.setPower(0);
                rf.setPower(0);
                lb.setPower(0);
                exit = false;
            }
            else
                calculatePower(speeds[1],speeds[0],  -1 * pidGyro.update(getAngle(firstGyro)), 1);
            packet.put("angle", currentAngle);
            packet.put("lf", lf.getCurrentPosition());
            packet.put("rf", rf.getCurrentPosition());
            packet.put("lb", lb.getCurrentPosition());
            packet.put("rb", rb.getCurrentPosition());
            packet.put("tar", wantedTicks);
            packet.put("y",wanted - odometry);
            packet.put("x",wanted - odometry);
            packet.put("error",wantedgyro-getAngle(firstGyro));
            packet.put("real", wantedTicks - odometry);
            packet.put("odometry", odometry);
            dash.sendTelemetryPacket(packet);

        }
    }
}

//    public void shoot(Robot robot,LinearOpMode linearOpMode, Shooter shooter, double angle1, double angle2, double angle3, double firstGyro, FtcDashboard dash, TelemetryPacket packet){
//        while (shooter.calculateWantedTPS(4) -shooter.averageTPS() <=150){
//            shooter.setServo(false);
//            shooter.execute(4);
//        }
//        shooter.setServo(true);
//        l
//        robot.dt.gyroTurn(angle1,firstGyro,linearOpMode.telemetry,dash,packet);
//        robot.intake.intakeMotor.setPower(0.4);
//        linearOpMode.sleep(1500);
//
//    }
