package org.firstinspires.ftc.teamcode.OpModes.Autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.WobbleMode;
import org.firstinspires.ftc.teamcode.countRings;

@Autonomous(name = "Megido_shoot_right_red")

public class ShootRightRed extends AutoBase  {

    @Override
    protected void runOpModeInternal(Robot robot, double firstGyro, FtcDashboard dash, TelemetryPacket packet) {
        sleep(5000);
        //robot.dt.CmDrivePid2(0,0,-10,firstGyro,0.4,0.5,dash,packet,40);
        //robot.dt.gyroTurn(90,firstGyro,telemetry,packet,dash);
        robot.dt.CmDrivePid2(0,0,-114,firstGyro,0.4,0.6,dash,packet,40);
        robot.dt.gyroTurn(-70,firstGyro,telemetry,packet,dash);
        robot.shooter.shoot(5,this,5000,robot);
        robot.dt.gyroTurn(-80,firstGyro,telemetry,packet,dash);
        robot.dt.CmDrivePid2(0,-80,-135,firstGyro,0.4,0.6,dash,packet,40);
        robot.dt.gyroTurn(10,firstGyro,telemetry,packet,dash);
        /*robot.wobble.set(WobbleMode.HOLD_WOBBLE_AND_MOVE);
        robot.wobble.execute(0);
        robot.dt.CmDrivePid2(0,0,-65,firstGyro,0.3,0.5,dash,packet,40);
        robot.dt.CmDrivePid2(0,-22,-50,firstGyro,0.4,0.5,dash,packet,40);
        //robot.dt.gyroTurn(-20,firstGyro,telemetry,packet,dash);
        robot.shooter.shoot(3,this,5000,robot);
        robot.dt.CmDrivePid2(0,robot.dt.getAngle(firstGyro),-120,firstGyro,0.4,0.5,dash,packet,40);
        robot.dt.gyroTurn(-160,firstGyro,telemetry,packet,dash);
        robot.wobble.set(WobbleMode.PUT_WOBBLE_DOWN);
        robot.wobble.execute(0);
        robot.wobble.set(WobbleMode.STATIC);
        robot.wobble.execute(0);
        sleep(1000);

        robot.dt.CmDrivePid2(0,-170,-90,firstGyro,0.3,0.6,dash,packet,40);
        robot.dt.gyroTurn(65,firstGyro,telemetry,packet,dash);
        /*
        switch (countRings.countRing(0)) {
            case 1:
                //back
                robot.wobble.set(WobbleMode.HOLD_WOBBLE_AND_MOVE);
                robot.dt.CmDrivePid2(0,-30,-250,firstGyro,0.3,0.7,dash,packet,40);//80
                robot.dt.gyroTurn(-160,firstGyro,telemetry,packet,dash);
                robot.wobble.set(WobbleMode.PUT_WOBBLE_DOWN);
                robot.wobble.execute(0);
                robot.wobble.set(WobbleMode.STATIC);
                robot.wobble.execute(0);
                sleep(1000);
                //robot.dt.CmDrivePid2(0,-160,70,firstGyro,0.4,0.7,dash,packet,40);
                //robot.dt.gyroTurn(70,firstGyro,telemetry,packet,dash);
            case 2:
                //middle
            case 3:
                //last

         */
        }
    }
