package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.WobbleMode;
@Autonomous(name = "shoot_left_red")
    public class ShootLeftRed extends AutoBase  {
        @Override
        protected void runOpModeInternal(Robot robot, double firstGyro, FtcDashboard dash, TelemetryPacket packet) {
            //sleep(10000);
            /*
            robot.wobble.set(WobbleMode.HOLD_WOBBLE_AND_MOVE);
            robot.wobble.execute(0);
            robot.dt.CmDrivePid(10,-0.4,firstGyro ,firstGyro,dash,packet);
            robot.dt.CmDrivePid2(0,-20,-130,firstGyro,0.4,0.7,dash,packet,40);
            robot.dt.gyroTurn(-110,firstGyro,telemetry,packet,dash);
            robot.dt.CmDrivePid2(0,-110,-20,firstGyro,0.3,0.5,dash,packet,40);
            robot.dt.gyroTurn(-20,firstGyro,telemetry,packet,dash);
            robot.dt.CmDrivePid2(0,-20,20,firstGyro,0.3,0.5,dash,packet,40);
            robot.shooter.shoot(3,this,4500,robot);
            robot.dt.CmDrivePid2(0,robot.dt.getAngle(firstGyro),-120,firstGyro,0.4,0.5,dash,packet,40);
            robot.dt.gyroTurn(-160,firstGyro,telemetry,packet,dash);
            robot.wobble.set(WobbleMode.PUT_WOBBLE_DOWN);
            robot.wobble.execute(0);
            robot.wobble.set(WobbleMode.STATIC);
            robot.wobble.execute(0);
            sleep(1000);
            robot.dt.CmDrivePid2(0,-160,-60,firstGyro,0.3,0.6,dash,packet,40);
            robot.dt.gyroTurn(70,firstGyro,telemetry,packet,dash);

             */

        }
    }
