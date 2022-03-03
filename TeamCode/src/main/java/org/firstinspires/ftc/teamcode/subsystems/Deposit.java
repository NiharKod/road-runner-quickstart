package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Deposit {

    Slides slides;

    DcMotorEx linkage;
    Gamepad gamepad;
    ElapsedTime timer;
    Telemetry telemetry;

    Servo armSecond;
    Servo claw;
    Servo armMain;

    public LinkageState linkageState = LinkageState.HIGH;

    public static double kp = 0.00425;
    public static double ki = 0.11;
    public static double kd = 0;
    public static int highLinkageTicks = 100;
    public static int midLinkageTicks = 50;
    public static int lowLinkageTicks = 25;
    public static int setPoint = 350;

    public static double resetMain = 0.01;
    public static double extend1Main = .025;
    public static double midMain = .87;
    public static double extend3Main =  .2;

    PIDController controller = new PIDController(kp,ki,kd);


    public Deposit(HardwareMap hardwareMap, Gamepad gamepad1, ElapsedTime timer, Telemetry telemetry){
        slides = new Slides(hardwareMap);
        armMain = hardwareMap.get(Servo.class,"armMain");
        armSecond = hardwareMap.get(Servo.class,"armSecond");
         claw = hardwareMap.get(Servo.class, "bucket");
        linkage = hardwareMap.get(DcMotorEx.class, "linkage");
        linkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.gamepad = gamepad1;
        this.timer = timer;
        this.telemetry = telemetry;
    }

    public void extendArm3(){armMain.setPosition(extend3Main); }
    public void extendArmMid(){armMain.setPosition(midMain);}
    public void resetMain(){armMain.setPosition(resetMain);}


    public enum LinkageState {
        HIGH,
        MIDDLE,
        LOW;
    }

    public enum State{
        START,
        START_AUTO,
        RESET,
        IDLE,

    }
    public enum StateR{
        START,
        RESET_AUTO,
        RESET,
        IDLE,
    }

    public static int open = 0;
    public static int close = 0;


    public static State state = State.IDLE;
    public static StateR stateR = StateR.IDLE;

    public void closeClaw(){
        claw.setPosition(close);
    }
    public void openClaw(){
        claw.setPosition(open);
    }

    public void linkageUpdate(){
        if(state.equals(LinkageState.HIGH)){
            linkage.setPower(controller.calculate(highLinkageTicks,linkage.getCurrentPosition()));
        }else if(state.equals(LinkageState.MIDDLE)){
            linkage.setPower(controller.calculate(midLinkageTicks,linkage.getCurrentPosition()));
        }else if(state.equals(LinkageState.LOW)){
            linkage.setPower(controller.calculate(lowLinkageTicks,linkage.getCurrentPosition()));
        }
    }


    public void update(){
        if(gamepad.a || state.equals(State.START_AUTO)){
            state = State.START;
            timer.reset();
        }
        switch (state){
            case START:

        }
        linkageUpdate();
        slides.liftUpdate();
    }
}
