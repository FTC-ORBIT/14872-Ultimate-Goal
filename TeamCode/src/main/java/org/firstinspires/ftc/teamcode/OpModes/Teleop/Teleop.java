package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;

import static org.firstinspires.ftc.teamcode.Utilities.wrapAnglePlusMinus180;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "FieldTeleop")
public class Teleop extends OpMode {
    Robot robot;
    double firstGyro;
    private Camera cam;

    @Override
    public void init() {
        cam = new Camera();
        robot = new Robot(this);
        cam.init(hardwareMap,telemetry);
        robot.init();
    }
    public void start(){
        firstGyro = wrapAnglePlusMinus180(wrapAnglePlusMinus180(robot.dt.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle));
    }
    @Override
    public void loop() {
      robot.Teleop(firstGyro);
      FtcDashboard.getInstance().startCameraStream(cam.cam,40);
    }
}