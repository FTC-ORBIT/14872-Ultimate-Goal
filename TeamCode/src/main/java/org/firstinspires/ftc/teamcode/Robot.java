package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.IntakeMode;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.RobotMode;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.ShooterMode;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.TransferMode;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.WobbleMode;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Wobble;

public class Robot {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();
    private Telemetry telemetry;
    private HardwareMap hwMap;
    private Gamepad driver;
    private Gamepad commander;

    private int level =1;
    private boolean isTurning = false;

    public Transfer transfer = new Transfer();
    public Intake intake = new Intake();
    public DriveTrain dt =new DriveTrain();
    public Wobble wobble = new Wobble();
    public Shooter shooter = new Shooter();

    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    double wantedGyro = 0;

    private RobotMode robotMode = RobotMode.TRAVEL;
    private TransferMode transferMode = TransferMode.STATIC;
    private ShooterMode shootingMode = ShooterMode.STATIC;
    private IntakeMode intakeMode =IntakeMode.STATIC;
    private WobbleMode wobbleMode = WobbleMode.STATIC;


    public Robot(OpMode opMode){
        this.hwMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.driver = opMode.gamepad1;
        this.commander = opMode.gamepad2;
    }
    public void init() {
        shooter.init(hwMap);

        transfer.init(hwMap);

        intake.init(hwMap);

        wobble.init(hwMap);

        dt.init(hwMap);
    }
    private boolean hasServoBeenOpen = false;

    public void subSystemManager(double x, double y, double r, double firstGyro,int level) {
        switch (robotMode) {
            case TRAVEL:
                transferMode = TransferMode.STATIC;
                shootingMode = ShooterMode.STATIC;
                intakeMode = IntakeMode.STATIC;
                wobbleMode = WobbleMode.STATIC;
                dt.fieldCentric(x,y,r,Constant.speedFactor,firstGyro);
                hasServoBeenOpen = false;
                break;
            case OUTPUT:
                transferMode = TransferMode.DEPLETE;
                shootingMode = ShooterMode.STATIC;
                intakeMode = IntakeMode.DEPLETE;
                wobbleMode = WobbleMode.STATIC;
                dt.fieldCentric(x,y,r,Constant.speedFactor,firstGyro);
                hasServoBeenOpen = false;
                break;
            case SHOOTING:

                wobbleMode = WobbleMode.STATIC;
                if (Math.abs(shooter.calculateWantedTPS(level) - shooter.averageTPS()) <= Constant.tpsError) {
                    hasServoBeenOpen = true;
                }
                if (hasServoBeenOpen) {
                    shootingMode = ShooterMode.SHOOT_OPEN;
                    transferMode = TransferMode.TRANSFER;
                    intakeMode = IntakeMode.COLLECTION;
                }
                else {
                    shootingMode = ShooterMode.SHOOT_CLOSED;
                    transferMode = TransferMode.STATIC;
                    intakeMode = IntakeMode.STATIC;
                }
                dt.fieldCentric(x,y,r/2,Constant.speedFactor,firstGyro);
                break;
            case INTAKE:
                transferMode = TransferMode.TRANSFER;
                shootingMode = ShooterMode.STATIC;
                intakeMode = IntakeMode.COLLECTION;
                wobbleMode = WobbleMode.STATIC;
                dt.fieldCentric(x,y,r,Constant.speedFactor,firstGyro);
                hasServoBeenOpen = false;
                break;
            case WOBBLE:
                transferMode = TransferMode.STATIC;
                shootingMode = ShooterMode.STATIC;
                intakeMode = IntakeMode.STATIC;
                switch (level){
                    case 1:
                        wobbleMode = WobbleMode.STATIC;
                        break;
                    case 2:
                        wobbleMode = WobbleMode.HOLD_WOBBLE_AND_MOVE;
                        break;
                    case 3:
                        wobbleMode =WobbleMode.HOLD_WOBBLE_IN_PLACE;
                        break;
                }
                dt.fieldCentric(x,y,r/2,Constant.speedFactor,firstGyro);
                hasServoBeenOpen = false;
                break;
        }

        shooter.set(shootingMode);
        transfer.set(transferMode);
        intake.set(intakeMode);
        wobble.set(wobbleMode);
    }
    public void Teleop(double  firstGyro){

        if (driver.a) robotMode = RobotMode.TRAVEL;
        if (driver.b ) robotMode = RobotMode.SHOOTING;
        if (driver.x) robotMode = RobotMode.INTAKE;
        if (driver.y) robotMode = RobotMode.OUTPUT;
        if (driver.right_stick_button) robotMode = RobotMode.WOBBLE;
        if (driver.dpad_down)wantedGyro = dt.getAngle(firstGyro);
        if (driver.dpad_right)isTurning = true;
        //if (robotMode.equals(RobotMode.SHOOTING) && !robotMode.equals(lastRobotMode)) firstTime = currentTime;

        double x = driver.left_stick_x;
        double y = driver.left_stick_y;
        double r = driver.right_trigger - driver.left_trigger * 1.82;

        double vexPower = driver.right_stick_y;

        boolean currentRightBumper = driver.right_bumper;
        boolean currentLeftBumper = driver.left_bumper;
        if (currentRightBumper == true && currentRightBumper != lastRightBumper) level++;
        if (currentLeftBumper == true && currentLeftBumper != lastLeftBumper) level--;

        if (robotMode == RobotMode.SHOOTING) {
            if (level > 6) level = 0;
            if (level < 0) level = 6;
        }else if(robotMode == RobotMode.WOBBLE){
            if (level > 3) level = 1;
            if (level < 1) level = 3;
        }

        double error = 90-dt.getAngle(wantedGyro);

        if (isTurning){
            if (Math.abs(error)>3){
                r=(3*error)/50;
            }else isTurning = false;
        }
        subSystemManager(x,y,r,wantedGyro,level);

        shooter.execute(level);

        transfer.execute();

        intake.execute();

        wobble.execute(vexPower);

        double[] powers = dt.getPowers();
        double[] positions = dt.getPositions();
        double[] velocities = shooter.getVelocity();

        telemetry.addData("level",level);
        telemetry.addData("robot state", robotMode);
        telemetry.addLine("modes")
                .addData(" intake",intakeMode)
                .addData("transfer",transferMode)
                .addData("shooter",shootingMode)
                .addData("wobble",wobbleMode);
        telemetry.addLine("dt powers")
                .addData(" lf", powers[0])
                .addData("rf",powers[1])
                .addData("lb",powers[2])
                .addData("rb",powers[3]);
        telemetry.addLine("dt positions")
                .addData(" lf",positions[0])
                .addData("rf",positions[1])
                .addData("lb",positions[2])
                .addData("rb",positions[3]);
        telemetry.addData("angle",dt.getAngle(wantedGyro));
        telemetry.addLine("")
                .addData(" intake power",intake.getPower())
                .addData("transfer power",transfer.getPower())
                .addData("wobble power", wobble.getPower());
        telemetry.addData("wanted shooting velocity",shooter.calculateWantedTPS(level));
        telemetry.addLine("velocities")
                .addData(" ls",velocities[0])
                .addData("rs",velocities[1]);
        telemetry.addLine("positions")
                .addData("shooter blocker", shooter.getShootingBlockerPosition())
                .addData("wobble claw position", wobble.getWobbleClawPosition());
        telemetry.update();
        packet.put("level", level);

        packet.put("firstGyro",firstGyro);
        packet.put("wanted", shooter.calculateWantedTPS(level));
        packet.put("current", shooter.averageTPS());
        dash.sendTelemetryPacket(packet);
        lastLeftBumper = currentLeftBumper;
        lastRightBumper = currentRightBumper;
    }
}