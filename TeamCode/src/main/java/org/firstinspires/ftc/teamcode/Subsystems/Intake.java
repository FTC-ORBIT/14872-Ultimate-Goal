package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.IntakeMode;

public final class Intake {

    private DcMotor intakeMotor;
    private double power = 0;

    //sets intake subsystem state
    public void set(final IntakeMode state) {

        switch (state) {
            default:
            case STATIC:
                power = 0;
                break;
            case COLLECTION:
                power = Constant.intakeCollectingPower;
                break;
            case DEPLETE:
                power = Constant.intakeDepletionPower          ;
        }
    }

    //give intake motor power according to subsystem state
    public void execute(){
        intakeMotor.setPower(power);
    }

    //intake subsystem initiation
    public void init(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class, "IM");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public double getVexTicks(){
        return intakeMotor.getCurrentPosition();
    }
    //returns intake motor power
    public double getPower(){
        return intakeMotor.getPower();
    }
}