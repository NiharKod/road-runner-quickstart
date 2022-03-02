package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Desposit {

    Slides slides;
    Servo armServo1, armServo2;
    Servo claw;
    DcMotorEx linkage;
    Gamepad gamepad;
    ElapsedTime timer;
    Telemetry telemetry;

    public Deposit(HardwareMap hardwareMap, Gamepad gamepad1, ElapsedTime timer, Telemetry telemetry){
        slides = new Slides(hardwareMap);
        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        armServo2 = hardwareMap.get(Servo.class, "armServo2");
        linkage = hardwareMap.get(DcMotorEx.class, "linkage");
        linkage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linkage.setTargetPositionTolerance(5);
        this.gamepad = gamepad1;
        this.timer = timer;
        this.telemetry = telemetry;
    }



    public enum LinkageState {
        HIGH,
        MIDDLE,
        LOW;
    }

    LinkageState linkageState = LinkageState.HIGH;

    public void linkageMove(LinkageState state){
        if(state.equals(LinkageState.HIGH)){
            linkage.setTargetPosition(100);
            linkage.setPower(.8);
            
        }
    }
}
