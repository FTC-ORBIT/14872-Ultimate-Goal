package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.IntakeMode;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.ShooterMode;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.TransferMode;

import static org.firstinspires.ftc.teamcode.Utilities.average2;

public class Shooter {
    private DcMotorEx leftShoot;
    private DcMotorEx rightShoot;
    private Servo shootingBlocker;

    private double shootingBlockerPosition;
    private double power = 0;
    private PID pid = new PID(Constant.shooterKP,Constant.shooterKI,Constant.shooterKD,Constant.shooterKF,Constant.shooterIZone);


    //sets shooter subsystem state
    public void set(final ShooterMode state){
        switch(state){
            case STATIC:
               power = 0;
                shootingBlockerPosition = Constant.transferServoClosed;
                break;
            case SHOOT_CLOSED:
               power = 1;
                shootingBlockerPosition = Constant.transferServoClosed;
                break;
            case SHOOT_OPEN:
                power = 1;
                shootingBlockerPosition = Constant.transferServoOpen;
                break;
        }
    }

    //returns average ticks per second
    public double averageTPS(){
        return  average2(leftShoot.getVelocity(),rightShoot.getVelocity());
    }

    //calculates wanted ticks per second according to driver bumpers
    public int calculateWantedTPS(int level){
        int target = 0;
        switch (level){
            case 1:
                target = Constant.lowGoalTPS;
                break;
            case 2:
                target= Constant.middleGoalTPS;
                break;
            case 3:
                target = Constant.highGoalTPS;
                break;
            case 4:
                target = Constant.powerShotTPS;
                break;
            case 0:
                target = 0;
                break;
            case  5:
                target = Constant.midgoallong;
                break;
            case 6:
                target = Constant.longhighgoal;
        }
        return target;
    }

    //calculates shooting motor velocities using pidf see PID class
    private double calculateVelocity(double wantedTPS) {

        pid.kP = Constant.shooterKP;
        pid.kI = Constant.shooterKI;
        pid.kD = Constant.shooterKD;
        pid.iZone = Constant.shooterIZone;
        pid.kF = Constant.shooterKF;
        pid.setWanted(wantedTPS);
        return pid.update(averageTPS());
    }

    //returns motor current velocities in ticks per second
    public double[] getVelocity() {
        double[] velocities = {leftShoot.getVelocity(),rightShoot.getVelocity()};
        return velocities;
    }

    //returns shooting blocker current position
    public double getShootingBlockerPosition() {
        return shootingBlocker.getPosition();
    }

    //gives shooter subsystem motors power and positions according to subsystem state and driver level
    public void execute(int level){
        leftShoot.setPower(power*calculateVelocity(calculateWantedTPS(level)));
        rightShoot.setPower(power*calculateVelocity(calculateWantedTPS(level)));
        shootingBlocker.setPosition(shootingBlockerPosition);
    }
    private void execute2(int level){
        leftShoot.setPower(calculateVelocity(calculateWantedTPS(level)));
        rightShoot.setPower(calculateVelocity(calculateWantedTPS(level)));
        shootingBlocker.setPosition(shootingBlockerPosition);
    }

    private void setServoOpen(boolean OpenOrClosed){
        if (OpenOrClosed){
            shootingBlocker.setPosition(Constant.transferServoOpen);
        }else shootingBlocker.setPosition(Constant.transferServoClosed);
    }

    //shooter subsystem initiation
    public void init(HardwareMap hardwareMap){
        shootingBlocker = hardwareMap.get(Servo.class, "TS");
        leftShoot = hardwareMap.get(DcMotorEx.class, "LS");
        rightShoot = hardwareMap.get(DcMotorEx.class, "RS");
        leftShoot.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShoot.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void shoot(int level,LinearOpMode linearOpMode,long timeShooting,Robot robot){
        boolean hasServoBeenOpen = false;
        ShooterMode shooterMode;
        TransferMode transferMode;
        IntakeMode intakeMode;
        while (true) {
            if (Math.abs(calculateWantedTPS(level) - averageTPS()) <= Constant.tpsError) {
                hasServoBeenOpen = true;
            }
            if (hasServoBeenOpen) {
                linearOpMode.sleep(2000);
                shooterMode = ShooterMode.SHOOT_OPEN;
                transferMode = TransferMode.TRANSFER;
                intakeMode = IntakeMode.COLLECTION;
                set(shooterMode);
                robot.intake.set(intakeMode);
                robot.transfer.set(transferMode);
                execute(level);
                robot.intake.execute();
                robot.transfer.execute();
                linearOpMode.sleep(timeShooting);
                transferMode = TransferMode.STATIC;
                intakeMode = IntakeMode.STATIC;
                shooterMode = ShooterMode.STATIC;
                set(shooterMode);
                robot.intake.set(intakeMode);
                robot.transfer.set(transferMode);
                execute(level);
                robot.intake.execute();
                robot.transfer.execute();
                break;
            } else {
                shooterMode = ShooterMode.SHOOT_CLOSED;
                transferMode = TransferMode.STATIC;
                intakeMode = IntakeMode.STATIC;
            }
            set(shooterMode);
            robot.intake.set(intakeMode);
            robot.transfer.set(transferMode);
            execute2(level);
            robot.intake.execute();
            robot.transfer.execute();

        }
        }


    public void powerShotShoot(LinearOpMode linearOpMode, Robot robot, double firstTurn, double secondTurn, double firstGyro, int waitBetweenShots, Telemetry telemetry, TelemetryPacket packet, FtcDashboard dash){
        execute2(5);
        while (true){
            if (calculateWantedTPS(5)-averageTPS()<= 150){
                linearOpMode.sleep(waitBetweenShots);
                linearOpMode.sleep(1000);
                setServoOpen(true);
                robot.transfer.set(TransferMode.TRANSFER);
                robot.intake.set(IntakeMode.COLLECTION);
                robot.transfer.execute();
                robot.intake.execute();
                linearOpMode.sleep(waitBetweenShots);


                robot.transfer.set(TransferMode.STATIC);
                robot.intake.set(IntakeMode.STATIC);
                robot.transfer.execute();
                robot.intake.execute();
                robot.dt.gyroTurn(firstTurn,firstGyro,telemetry,packet,dash);
                robot.transfer.set(TransferMode.TRANSFER);
                robot.intake.set(IntakeMode.COLLECTION);
                robot.transfer.execute();
                robot.intake.execute();
                linearOpMode.sleep(waitBetweenShots);

                robot.transfer.set(TransferMode.STATIC);
                robot.intake.set(IntakeMode.STATIC);
                robot.transfer.execute();
                robot.intake.execute();
                robot.dt.gyroTurn(secondTurn,firstGyro,telemetry,packet,dash);

                robot.transfer.set(TransferMode.TRANSFER);
                robot.intake.set(IntakeMode.COLLECTION);
                robot.transfer.execute();
                robot.intake.execute();
                linearOpMode.sleep(waitBetweenShots);
                break;
            }else {
                setServoOpen(false);}
        }
        execute(0);
        setServoOpen(false);
    }
}


