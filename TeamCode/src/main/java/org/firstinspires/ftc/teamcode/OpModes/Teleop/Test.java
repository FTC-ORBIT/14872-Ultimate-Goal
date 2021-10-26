package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;


@TeleOp(name = "test")
public class Test extends OpMode {
    private Camera cam = new Camera();

    @Override
    public void init() {
        cam.init(hardwareMap,telemetry);
    }

    @Override
    public void loop(){
        FtcDashboard.getInstance().startCameraStream(cam.cam,40);
    }
}
