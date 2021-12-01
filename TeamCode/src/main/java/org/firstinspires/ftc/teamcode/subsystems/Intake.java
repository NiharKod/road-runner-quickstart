package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    HardwareMap hardwareMap;
    public Intake(HardwareMap hw){
        hardwareMap = hw;
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void stop(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
    public void setPower(double power){
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
}
