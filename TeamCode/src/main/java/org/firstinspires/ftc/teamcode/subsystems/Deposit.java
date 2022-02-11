package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Deposit {
   public Servo claw;
   public  Servo virtualFourBar;
   public Servo wrist;
    Slides slides;
    Intake intake;
    Gamepad gamepad;
    ElapsedTime timer;
    Telemetry telemetry;
    public static double open = .55;
    public static double close = .43;

    public static double armIdle = 0.5;
    public static double armIntake = .2;
    public static double armLevelOne = 0.82;
    public static double armLevelThree = 0.64;

    public static double wristIntake = .77;
    public static double wristDeposit = 0.02;
    public static double wristIdle = .72;

    public Deposit(HardwareMap hardwareMap, Gamepad gamepad1, ElapsedTime timer, Telemetry telemetry){
        claw = hardwareMap.get(Servo.class, "claw");
        virtualFourBar = hardwareMap.get(Servo.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        intake = new Intake(hardwareMap);
        slides = new Slides(hardwareMap);
        gamepad = gamepad1;
        this.timer = timer;
        this.telemetry = telemetry;

    }
    public void closeClaw(){
        claw.setPosition(close);
    }
    public void openClaw(){
        claw.setPosition(open);
    }
    public void armIntake(){ virtualFourBar.setPosition(armIntake); }
    public void armIdle(){
        virtualFourBar.setPosition(armIdle);
    }

    public void armLvlThree(){
        virtualFourBar.setPosition(armLevelThree);
    }

    public void wristIdle(){ wrist.setPosition(wristIdle); }
    public void wristDeposit(){
        wrist.setPosition(wristDeposit);
    }
    public void wristIntake(){ wrist.setPosition(wristIntake);}

    public enum State{
        START,
        START_AUTO,
        OPEN_CLAW,
        RESET,
        IDLE,
        ARM_IDLE,
        WRIST_DEPOSIT,
        CLOSE_CLAW,
        STOP_INTAKE,
        SLIDES_LIFT,
        SLIDES_RESET,
        ARM_SCORING,
        WRIST_IDLE,
        ARM_INTAKE,
        DUMP
    }
    public enum StateR{
        START,
        RESET_AUTO,
        OPEN_CLAW,
        RESET,
        IDLE,
        ARM_IDLE,
        WRIST_DEPOSIT,
        CLOSE_CLAW,
        STOP_INTAKE,
        SLIDES_LIFT,
        SLIDES_RESET,
        ARM_SCORING,
        WRIST_IDLE,
        ARM_INTAKE,
        WRIST_INTAKE
    }

    public static State state = State.ARM_IDLE;
    public static StateR stateR = StateR.ARM_IDLE;




    public void updateAuto(){
        if(state.equals(State.START_AUTO)){
            state = State.START;
            timer.reset();
       }
        switch(state){
            case START:
                closeClaw();
                if(timer.milliseconds() > 150){
                    state = State.WRIST_IDLE;
                }
                break;
            case WRIST_IDLE:
                wristIdle();
                if(timer.milliseconds() > 300){
                    state = State.ARM_IDLE;
                }
                break;
            case ARM_IDLE:
                armIdle();
                if(timer.milliseconds() > 600){
                    state = State.WRIST_DEPOSIT;
                    intake.enable();
                }
                break;
            case WRIST_DEPOSIT:
                wristDeposit();
                if(timer.milliseconds() > 750){
                    state = State.SLIDES_LIFT;
                }
                break;
            case SLIDES_LIFT:
                Slides.state = Slides.State.LIFT;
                if(timer.milliseconds() > 1000){
                    state = State.DUMP;
                    timer.reset();
                }
                break;
            case DUMP:
                armLvlThree();
                state = State.STOP_INTAKE;
                return;

        }
        slides.liftUpdate();
    }

    public void update(){
        if(gamepad.x){
            state = State.START;
            timer.reset();
        }
        switch(state){
            case START:
                closeClaw();
                if(timer.milliseconds() > 150){
                    state = State.WRIST_IDLE;
                }
                break;
            case WRIST_IDLE:
                wristIdle();
                if(timer.milliseconds() > 300){
                    state = State.ARM_IDLE;
                }
                break;
            case ARM_IDLE:
                armIdle();
                if(timer.milliseconds() > 600){
                    state = State.WRIST_DEPOSIT;
                    intake.enable();
                }
                break;
            case WRIST_DEPOSIT:
                wristDeposit();
                if(timer.milliseconds() > 750){
                    state = State.SLIDES_LIFT;
                }
                break;
            case SLIDES_LIFT:
                Slides.state = Slides.State.LIFT;
                if(timer.milliseconds() > 1000){
                    state = State.DUMP;
                    timer.reset();
                }
                break;
            case DUMP:
                armLvlThree();
                state = State.STOP_INTAKE;
                return;

        }
        slides.liftUpdate();
    }

    public void resetUpdateAuto(){
        if(stateR.equals(StateR.RESET_AUTO)){
            stateR = StateR.RESET;
            timer.reset();
        }
        switch(stateR){
            case RESET:
                openClaw();
                if(timer.milliseconds() > 150){
                    stateR = StateR.ARM_IDLE;
                }
                break;
            case ARM_IDLE:
                armIdle();
                if(timer.milliseconds() > 300){
                    stateR = StateR.WRIST_IDLE;
                }
                break;
            case WRIST_IDLE:
                wristIdle();
                if(timer.milliseconds() > 600){
                    stateR = StateR.SLIDES_RESET;
                }
                break;
            case SLIDES_RESET:
                Slides.state = Slides.State.RESET;
                if(timer.milliseconds() > 750){
                    stateR = StateR.CLOSE_CLAW;
                }
                break;
            case CLOSE_CLAW:
                closeClaw();
                if(timer.milliseconds() > 900){
                    stateR = StateR.ARM_INTAKE;
                }
                break;
            case ARM_INTAKE:
                armIntake();
                if(timer.milliseconds() > 1050){
                    stateR = StateR.OPEN_CLAW;
                }
                break;
            case OPEN_CLAW:
                openClaw();
                if(timer.milliseconds() > 1400){
                    stateR = StateR.WRIST_INTAKE;
                }
                break;
            case WRIST_INTAKE:
                wristIntake();
                if(timer.milliseconds() > 1500){
                    stateR = StateR.IDLE;
                }
                return;
        }
        slides.liftUpdate();
    }


    public void resetUpdate(){
        if(gamepad.b){
            stateR = StateR.RESET;
            timer.reset();
        }
        switch(stateR){
            case RESET:
                openClaw();
                if(timer.milliseconds() > 150){
                    stateR = StateR.ARM_IDLE;
                }
                break;
            case ARM_IDLE:
                armIdle();
                if(timer.milliseconds() > 300){
                    stateR = StateR.WRIST_IDLE;
                }
                break;
            case WRIST_IDLE:
                wristIdle();
                if(timer.milliseconds() > 600){
                    stateR = StateR.SLIDES_RESET;
                }
                break;
            case SLIDES_RESET:
                Slides.state = Slides.State.RESET;
                if(timer.milliseconds() > 750){
                    stateR = StateR.CLOSE_CLAW;
                }
                break;
            case CLOSE_CLAW:
                closeClaw();
                if(timer.milliseconds() > 900){
                    stateR = StateR.ARM_INTAKE;
                }
                break;
            case ARM_INTAKE:
                armIntake();
                if(timer.milliseconds() > 1050){
                    stateR = StateR.OPEN_CLAW;
                }
                break;
            case OPEN_CLAW:
                openClaw();
                if(timer.milliseconds() > 1400){
                    stateR = StateR.WRIST_INTAKE;
                }
                break;
            case WRIST_INTAKE:
                wristIntake();
                if(timer.milliseconds() > 1500){
                    stateR = StateR.IDLE;
                }
                return;
        }
        slides.liftUpdate();
    }

}
