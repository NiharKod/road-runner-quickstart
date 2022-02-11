package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotorEx intake;
    HardwareMap hardwareMap;
    public Intake(HardwareMap hw){
        hardwareMap = hw;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void stop(){
        intake.setPower(0);
    }
    public void setPower(double power){
        intake.setPower(power);
    }
    public void disable(){intake.setMotorDisable();}
    public void enable(){intake.setMotorEnable();}

}
