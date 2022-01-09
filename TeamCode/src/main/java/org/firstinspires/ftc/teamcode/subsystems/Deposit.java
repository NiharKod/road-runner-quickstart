package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class Deposit {
   public Servo claw;
  public  Servo virtualFourBar;
   public Servo wrist;
    Slides slides;
    Intake intake;
    double currentTime = 0;
    Gamepad gamepad;
    ElapsedTime timer;
    public static double open = .6;
    public static double close = .43;
    public static double resetV4b = 0.2;
    public static double extendMid = 0.3;
    public static double extendReset = .2;
    public static double extendOne = 0.82;
    public static double extendthree = 0.55;

    public static double wristIdle = .78;
    public static double wrist2 = 0.08;

    public Deposit(HardwareMap hardwareMap, Gamepad gamepad1, ElapsedTime timer){
        claw = hardwareMap.get(Servo.class, "claw");
        virtualFourBar = hardwareMap.get(Servo.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        intake = new Intake(hardwareMap);
        gamepad = gamepad1;
        this.timer = timer;
    }
    public void closeClaw(){
        claw.setPosition(close);
    }
    public void openClaw(){
        claw.setPosition(open);
    }
    public void v4bReset(){
        virtualFourBar.setPosition(resetV4b);
    }
    public void v4bMid(){
        virtualFourBar.setPosition(extendMid);
    }

    public void v4bone(){
        virtualFourBar.setPosition(extendOne);
    }
    public void v4bthree(){
        virtualFourBar.setPosition(extendthree);
    }

    public void wristIdle(){
        wrist.setPosition(wristIdle);
    }
    public void wristDeposit(){
        wrist.setPosition(wrist2);
    }

    public enum State{
        START,
        IDLE,
        INTAKE_OFF,
        ARM_IDLE,
        WRIST_DEPOSIT,
        CLOSE_CLAW,
        LIFT_SLIDES,
        ARM_SCORING
    }

    State state = State.IDLE;


    public void update(){
        if(gamepad.x){
            state = State.START;
        }
        switch(state){
            case START:
                intake.stop();
                state = State.WRIST_DEPOSIT;
                break;
            case ARM_IDLE:
                timer.reset();
                v4bMid();
                if(timer.milliseconds() > 4000){
                    state = State.WRIST_DEPOSIT;
                }
                break;
            case WRIST_DEPOSIT:
                timer.reset();
                wristDeposit();
                if(timer.milliseconds() > 4000){
                    state = State.CLOSE_CLAW;
                }
                break;
            case CLOSE_CLAW:
                timer.reset();
                closeClaw();
                if(timer.milliseconds() > 3000){
                    state = State.LIFT_SLIDES;
                }
                break;
            case LIFT_SLIDES:
                Slides.state = Slides.State.LIFT;
            case ARM_SCORING:
                timer.reset();
                v4bthree();
                if(timer.milliseconds() > 1000){
                   state = State.IDLE;
                }
                return;

        }
        slides.lift();
    }


    public void reset(){
        
    }
}
