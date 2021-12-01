package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Turret {
    DcMotorEx turretMotor;
    HardwareMap hw;

    int ticksPerRot = 365;
    int ticksPerRev = 0;
    int ticksPerDeg = ticksPerRot / 360;

    public Turret(HardwareMap hw){
        this.hw = hw;
        turretMotor = hw.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,new PIDFCoefficients(1,1,1,1));
    }


    public void turnTo(double degrees){

    }

}
