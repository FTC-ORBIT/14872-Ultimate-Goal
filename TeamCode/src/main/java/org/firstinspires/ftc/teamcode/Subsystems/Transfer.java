package org.firstinspires.ftc.teamcode.Subsystems;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Modes.TransferMode;

public final class Transfer {

    private DcMotor transferMotor;

    private double power = 0;


    //sets transfer subsystem state
    public void set(final TransferMode state) {

        switch (state) {
            default:
            case STATIC:
                power = 0;
                break;
            case TRANSFER:
                power = Constant.transferringPower + ((Constant.transferringPower - transferMotor.getPower())  * Constant.pfortransfer);
                break;

            case DEPLETE:
                power = Constant.transferDepletingPower;
                break;
            case DEPLETE_SLOWLY:
                power = Constant.transferDepletingPower/4;
                break;
        }
    }

    //gives transfer subsystem motor power according to subsystem state
    public void execute(){
        transferMotor.setPower(power);
    }

    //wobble subsystem initiation
    public void init(HardwareMap hardwareMap){
        transferMotor = hardwareMap.get(DcMotor.class, "TM");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //returns transfer motor power
    public double getPower() {
        return transferMotor.getPower();
    }
}