package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Deposit {
    Servo claw;
    Servo virtualFourBar;
    Servo wrist;
    Slides slides;
    Intake intake;
    double currentTime = 0;
    Gamepad gamepad;
    ElapsedTime timer;
    public Deposit(HardwareMap hardwareMap, Gamepad gamepad1, ElapsedTime timer){
        claw = hardwareMap.get(Servo.class, "claw");
        virtualFourBar = hardwareMap.get(Servo.class, "virtualFourBar");
        wrist = hardwareMap.get(Servo.class, "wrist");
        slides = new Slides(hardwareMap);
        intake = new Intake(hardwareMap);
        gamepad = gamepad1;
        this.timer = timer;


    }
    private enum State{
        START,
        RESET,
        INTAKE_STOP,
        CLOSE_CLAW,
        OPEN_CLAW,
        EXTEND_LIFT,
        FLIP_V4B,
        RETRACT_V4B
    }

    State depositState = State.START;
    public void openClaw(){

    }
    public void closeClaw(){

    }
    public void v4bLOne(){

    }
    public void v4bTwo(){

    }
    public void v4bThree(){

    }

    public void updateGoalDepositing(){
        switch(depositState){
            case START:
                if(gamepad.x) {
                    intake.stop();
                    depositState = State.CLOSE_CLAW;
                    timer.reset();
                }
                break;
            case OPEN_CLAW:
                    //close claw
                    closeClaw();
                    //check if there has been enough time;
                    if(timer.seconds() >= .5){
                        depositState = State.EXTEND_LIFT;
                    }
                break;
            case EXTEND_LIFT:
                slides.liftTo(Slides.Level.THREE);
                if(slides.percentToTarget() >= 75){
                    depositState = State.FLIP_V4B;
                }
                break;
            case: 
        }
    }
}
