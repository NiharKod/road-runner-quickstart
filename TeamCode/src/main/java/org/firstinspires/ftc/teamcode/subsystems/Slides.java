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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
public class Slides {
    public DcMotorEx leftMotor;
    public DcMotorEx rightMotor;
    public DcMotorEx lockMotor;
    public Servo armMain;
    public Servo armSecond;
    public Servo claw;
    HardwareMap hw;
    public static double kp = 0.00425;
    public static double ki = 0.11;
    public static double kd = 0;
    public static int setPoint = 350;
    public static double powerMultiplier = .5;
    PIDController controller = new PIDController(kp,ki,kd);
    public Slides(HardwareMap hw){
        this.hw = hw;
        leftMotor = hw.get(DcMotorEx.class, "leftMotor");
        rightMotor = hw.get(DcMotorEx.class, "rightMotor");
       // lockMotor = hw.get(DcMotorEx.class, "lockMotor");
    //    lockMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public enum State{
        INACTIVE,
        LIFT,
        RESET
    }

    public static State state = State.INACTIVE;


    public void liftUpdate(){
     //   lockPosition();
        if(state == State.LIFT) {
            leftMotor.setPower(controller.calculate(-setPoint,leftMotor.getCurrentPosition()) * powerMultiplier);
            rightMotor.setPower(-controller.calculate(-setPoint,leftMotor.getCurrentPosition()) * powerMultiplier);
        }else if(state == State.RESET){
            leftMotor.setPower((controller.calculate(0,leftMotor.getCurrentPosition())) * powerMultiplier);
            rightMotor.setPower((-controller.calculate(0,leftMotor.getCurrentPosition())) * powerMultiplier);
        }
    }



    public void holdPosition(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void setPower(double power){
            leftMotor.setPower(power);
            rightMotor.setPower(-power);
    }

    public double percentToTarget(){
        return (leftMotor.getCurrentPosition() / controller.getSetPoint()) * 100;
    }

}
