package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Turret {
    DcMotorEx turretMotor;
    HardwareMap hw;

    private static int TICKS_PER_ROTATION = 312;
    private static int GEAR_RATIO = 120 / 40;
    private static int TICKS_PER_DEGREE = (TICKS_PER_ROTATION * GEAR_RATIO) / 360;

    public static double kp = .0001;
    public static double ki = 0;
    public static double kd = 0;
    PIDController controller = new PIDController(kp,ki,kd);

    public Turret(HardwareMap hw){
        this.hw = hw;
        turretMotor = hw.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }




    public void update(double degrees){
        double deg = TICKS_PER_DEGREE * degrees;
        turretMotor.setPower(controller.calculate(turretMotor.getCurrentPosition(), deg));
    }

    public void getCurrentAngle(){

    }

    public void reset(){

    }

}
