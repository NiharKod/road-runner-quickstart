package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Slides {
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    TouchSensor retractSensor;
    HardwareMap hw;
    PIDFController pidfController = new PIDFController(new PIDCoefficients(0,0,0));
    int targetTicks = 0;
    public Slides(HardwareMap hw){
        this.hw = hw;
        leftMotor = hw.get(DcMotorEx.class, "leftMotor");
        rightMotor = hw.get(DcMotorEx.class, "rightMotor");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    Level level = Level.NULL;



    public void lift(Level level){
        this.level = level;
        double ticks = 100;
        switch(level){
            case ONE: ticks = 1000; break;
            case TWO: ticks = 2000; break;
            case THREE: ticks = 3000; break;
        }
        pidfController.setTargetPosition(ticks);
        if(pidfController.getLastError() >= 3) {

            leftMotor.setPower(pidfController.update(leftMotor.getCurrentPosition()));
            rightMotor.setPower(pidfController.update(leftMotor.getCurrentPosition()));
        }else{
            holdPosition();
        }
    }

    public void reset(){
        pidfController.setTargetPosition(0);
        if(pidfController.getLastError() >= 3) {

            leftMotor.setPower(pidfController.update(leftMotor.getCurrentPosition()));
            rightMotor.setPower(pidfController.update(leftMotor.getCurrentPosition()));
        }else{
            holdPosition();
        }
    }

    public void holdPosition(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

}
