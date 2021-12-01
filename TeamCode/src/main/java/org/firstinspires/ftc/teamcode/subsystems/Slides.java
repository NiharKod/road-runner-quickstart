package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Slides {
    DcMotorEx motor;
    TouchSensor retractSensor;
    HardwareMap hw;
    PIDFController pidfController = new PIDFController(0,0,0,0);
    int targetTicks = 0;
    public Slides(HardwareMap hw){
        this.hw = hw;
        motor = hw.get(DcMotorEx.class, "slidesMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        retractSensor = hw.get(TouchSensor.class, "touch");
    }


        public void extend(double power){
            pidfController.setTargetPosition(targetTicks);
            motor.setPower(pidfController.update(motor.getCurrentPosition()));
        }

        public void retract(){
            if(retractSensor.isPressed()){
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }

}
