package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.WobbleMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CONNECTION_Shoot_Left_Blue")

public class ShootLeftBlue extends AutoBase {
    protected void runOpModeInternal(Robot robot, double firstGyro, FtcDashboard dash, TelemetryPacket packet) {
        sleep(5000);
        //robot.dt.CmDrivePid2(0,0,-10,firstGyro,0.4,0.5,dash,packet,40);
        //robot.dt.gyroTurn(90,firstGyro,telemetry,packet,dash);
        robot.dt.CmDrivePid2(0,0,-112,firstGyro,0.4,0.6,dash,packet,40);
        robot.dt.gyroTurn(70,firstGyro,telemetry,packet,dash);
        robot.shooter.shoot(5,this,5000,robot);
        robot.dt.gyroTurn(80,firstGyro,telemetry,packet,dash);
        robot.dt.CmDrivePid2(0,80,-135 ,firstGyro,0.4,0.6,dash,packet,40);
        robot.dt.gyroTurn(-10,firstGyro,telemetry,packet,dash);
    }
}
