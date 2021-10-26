package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Utilities.wrapAnglePlusMinus180;


@Autonomous(group = "AutoTest")
public class AutoTests extends AutoBase {

    @Override
    protected void runOpModeInternal(Robot robot, double firstGyro, FtcDashboard dash, TelemetryPacket packet) {
        robot.shooter.shoot(5,this,5000,robot);
    }

}

