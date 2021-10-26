package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;
import static org.firstinspires.ftc.teamcode.Utilities.wrapAnglePlusMinus180;

public abstract class AutoBase extends LinearOpMode {


    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        FtcDashboard dash = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        robot.init();
        waitForStart();
        double firstGyro = wrapAnglePlusMinus180(robot.dt.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        runOpModeInternal(robot,firstGyro,dash,packet);
    }

    protected abstract void runOpModeInternal(Robot robot,double firstGyro,FtcDashboard dash,TelemetryPacket packet);



//    protected void shoot(Robot robot,double shootingPower,long shootingTime,double transferPower){
//        robot.getTransferServo().setPosition(Constant.transferServoClosed);
//        robot.getTransferMotor().setPower(transferPower);
//        robot.getLeftShoot().setPower(shootingPower);
//        robot.getRightShoot().setPower(shootingPower);
//        sleep(2000);
//        robot.getTransferServo().setPosition(Constant.transferServoOpen);
//        sleep(shootingTime);
//        robot.getLeftShoot().setPower(0);
//        robot.getRightShoot().setPower(0);
//        robot.getTransferMotor().setPower(0);
//        robot.getTransferServo().setPosition(Constant.transferServoClosed);
//        sleep(500);
//    }
//
//    protected void intake(Robot robot,double intakePower,double transferPower,long transferringTime){
//        robot.getTransferServo().setPosition(Constant.transferServoClosed);
//        robot.getTransferMotor().setPower(transferPower);
//        robot.getIntakeMotor().setPower(intakePower);
//        sleep(transferringTime);
//    }




//    }

//    public  void CmDrivePid(Double wanted,Robot robot,Telemetry telemetry,double kpDrive,double kpGyro,double firstGyro){
//        double wantedGyroAngle = Helpers.getAngle(robot.getImu(),firstGyro);
//        double speed =1 ;
//        PID pidCm = new PID(kpDrive,0,0,0);
//        PID pidGyro = new PID(kpGyro,0,0,0);
//        pidCm.setWanted(wanted);
//        pidGyro.setWanted(wantedGyroAngle);
//        while (opModeIsActive()){
//            double currentAngle = Helpers.getAngle(robot.getImu(),firstGyro);
//            double current = Helpers.odometry(robot);
//            if (wanted - current <= 10){
//                break;
//            }
//            speed = pidCm.update(current);
//            telemetry.addData("speed", speed);
//            dt.setMotorPower(speed,speed,speed,speed,pidGyro.update(currentAngle),robot);
//
//            telemetry.addData("lfPower",robot.getLeftFrontDrive().getPower());
//            telemetry.addData("lfTicks",robot.getLeftFrontDrive().getCurrentPosition());
//
//            telemetry.addData("rfPower",robot.getRightFrontDrive().getPower());
//            telemetry.addData("rfTicks",robot.getRightFrontDrive().getCurrentPosition());
//
//            telemetry.addData("lbPower",robot.getLeftBackDrive().getPower());
//            telemetry.addData("lbTicks",robot.getLeftBackDrive().getCurrentPosition());
//
//            telemetry.addData("rbPower",robot.getRightBackDrive().getPower());
//            telemetry.addData("rbTicks",robot.getRightBackDrive().getCurrentPosition());
//
//            telemetry.addData("average",current);
//            telemetry.addData("error",wanted-current);
//            telemetry.addData("current angle",currentAngle);
//            telemetry.update();
//
//        }
//        Helpers.calculatePower(0,0,0,robot,0);
//        return;
//    }
}
