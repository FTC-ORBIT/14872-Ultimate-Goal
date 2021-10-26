package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.WobbleMode;


public final class Wobble {

    private CRServo wobbleArm;
    private Servo wobbleClaw;
    private double position;
    private double multiplier =0;
    private double add = 0;

    //sets wobble subsystem state
    public void set(final WobbleMode state){
        switch(state){
            case HOLD_WOBBLE_IN_PLACE:
                position = Constant.clawPositionClosed;
                multiplier = 0;
                add = Constant.vexHoldingPower;
                break;
            case STATIC:
                position = Constant.clawPositionOpened;
                multiplier = 0.2;
                add =0;
                break;
            case HOLD_WOBBLE_AND_MOVE:
                position = Constant.clawPositionClosed;
                multiplier = 1/1.1;
                add = 0;
                break;
            case PUT_WOBBLE_DOWN:
                position = Constant.clawPositionClosed;
                multiplier = 0;
                add = 1;
        }
    }

    //gives all wobble subsystem motors power and positions according to subsystem state
    public void execute(double gamepadPower){
        wobbleClaw.setPosition(position);
        wobbleArm.setPower((gamepadPower*multiplier) + add);
    }

    //wobble subsystem initiation
    public void init(HardwareMap hardwareMap){
        wobbleArm = hardwareMap.get(CRServo.class, "WM");
        wobbleClaw = hardwareMap.get(Servo.class, "WS");
    }

    //returns wobble arm power
    public double getPower() {
        return wobbleArm.getPower();
    }

    //returns wobble claw position
    public  double getWobbleClawPosition(){
        return wobbleClaw.getPosition();
    }
}