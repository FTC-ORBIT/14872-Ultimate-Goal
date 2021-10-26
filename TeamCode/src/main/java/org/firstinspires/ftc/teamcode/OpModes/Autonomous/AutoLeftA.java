package org.firstinspires.ftc.teamcode.OpModes.Autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.WobbleMode;

@Autonomous(name = "AutoLeftA")
public class AutoLeftA extends AutoBase {
    @Override
    protected void runOpModeInternal(Robot robot, double firstGyro, FtcDashboard dash, TelemetryPacket packet) {
        robot.dt.CmDrivePid(10,-0.4,firstGyro ,firstGyro,dash,packet);
        robot.dt.CmDrivePid2(0,-20,-148,firstGyro,0.5,0.8,dash,packet,40);
        robot.dt.gyroTurn(70,firstGyro,telemetry,packet,dash);
        robot.dt.CmDrivePid2(0,70,-58,firstGyro,0.3,0.5,dash,packet,40);
        robot.dt.gyroTurn(-20,firstGyro,telemetry,packet,dash);
        robot.shooter.powerShotShoot(this,robot, 3,5,firstGyro,500,telemetry,packet,dash);
    }

}
