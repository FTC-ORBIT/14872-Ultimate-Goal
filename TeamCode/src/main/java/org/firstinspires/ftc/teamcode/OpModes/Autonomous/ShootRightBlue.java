package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.WobbleMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Shoot_Right_Blue")
public class ShootRightBlue extends AutoBase {
    protected void runOpModeInternal(Robot robot, double firstGyro, FtcDashboard dash, TelemetryPacket packet) {
        robot.wobble.set(WobbleMode.HOLD_WOBBLE_AND_MOVE);
        robot.wobble.execute(0);
        robot.dt.CmDrivePid(10,-0.4,firstGyro ,firstGyro,dash,packet);
        robot.dt.CmDrivePid2(0,20,-130,firstGyro,0.4,0.7,dash,packet,40);
        robot.dt.gyroTurn(110,firstGyro,telemetry,packet,dash);
        robot.dt.CmDrivePid2(0,110,-40,firstGyro,0.3,0.5,dash,packet,40);
        robot.dt.gyroTurn(20,firstGyro,telemetry,packet,dash);
        robot.shooter.shoot(3,this,6000,robot);
        robot.dt.CmDrivePid2(0,robot.dt.getAngle(firstGyro),-120,firstGyro,0.4,0.5,dash,packet,40);
        robot.dt.gyroTurn(160,firstGyro,telemetry,packet,dash);
        robot.wobble.set(WobbleMode.PUT_WOBBLE_DOWN);
        robot.wobble.execute(0);
        robot.wobble.set(WobbleMode.STATIC);
        robot.wobble.execute(0);
        sleep(1000);
        robot.dt.CmDrivePid2(0,160,-60,firstGyro,0.3,0.6,dash,packet,40);
        robot.dt.gyroTurn(70,firstGyro,telemetry,packet,dash);
    }
}