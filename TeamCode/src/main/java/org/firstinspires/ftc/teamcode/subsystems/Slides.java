package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
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
    TouchSensor retractSensor;
    HardwareMap hw;
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;
    PIDFController pidfController = new PIDFController(new PIDCoefficients(kp,ki,kd));

    int targetTicks = 0;
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



    public void liftTo  (Level level){
        this.level = level;
        double ticks = 200;
        switch(level){
            case ONE: ticks = 0; break;
            case TWO: ticks = 0; break;
            case THREE: ticks = 200; break;
        }
        pidfController.setTargetPosition(ticks);

            leftMotor.setPower(pidfController.update(leftMotor.getCurrentPosition()));
            rightMotor.setPower(-pidfController.update(leftMotor.getCurrentPosition()));

    }

    public void reset(){
        pidfController.setTargetPosition(0);
        if(pidfController.getLastError() >= 3) {

            leftMotor.setPower(pidfController.update(leftMotor.getCurrentPosition()));
            rightMotor.setPower(pidfController.update(leftMotor.getCurrentPosition()));
        }else{
            holdPosition();
            state = State.INACTIVE;
        }
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
        return (leftMotor.getCurrentPosition() / pidfController.getTargetPosition()) * 100;
    }

}
