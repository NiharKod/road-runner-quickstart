package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
public class Slides {
    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;
    HardwareMap hw;
    public static double kp = 0.00425;
    public static double ki = 0.11;
    public static double kd = 0;
    PIDController controller = new PIDController(kp,ki,kd);
    public Slides(HardwareMap hw){
        this.hw = hw;
        leftMotor = hw.get(DcMotorEx.class, "leftMotor");
        rightMotor = hw.get(DcMotorEx.class, "rightMotor");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        controller.setTolerance(0);
    }

    public enum Level {
        NULL,
        ONE,
        TWO,
        THREE
    }

    public enum State{
        INACTIVE,
        REACHED_THRESHOLD,
        BUSY
    }

    Level level = Level.NULL;
    State state = State.INACTIVE;


    public void lift(){
        double ticks = 200;
        double gain = controller.calculate(200,leftMotor.getCurrentPosition());

        leftMotor.setPower(gain);
        rightMotor.setPower(-gain);
    }

    public void reset(){

    }

    public void holdPosition(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void setPower(double power){
        if(power != 0) {
            leftMotor.setPower(power);
            rightMotor.setPower(-power);
        }else {
            holdPosition();
        }

    }

    public double percentToTarget(){
        return (leftMotor.getCurrentPosition() / controller.getSetPoint()) * 100;
    }

}
