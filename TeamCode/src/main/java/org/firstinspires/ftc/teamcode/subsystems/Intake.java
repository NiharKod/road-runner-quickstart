package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake {
    DcMotorEx intake;
    HardwareMap hardwareMap;
    DistanceSensor distanceSensor;
    private boolean hasFreight = false;
    public static int distanceThreshold = 3;
    public Intake(HardwareMap hw) {
        hardwareMap = hw;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }


    public void stop() {
        intake.setPower(0);
    }

    public void setPower(double power) {
        if(distanceSensor.getDistance(DistanceUnit.CM) > distanceThreshold){
            intake.setPower(power);
        }
        else{
            intake.setPower(-power);
        }
    }

    public boolean hasFreight(){
        return distanceSensor.getDistance(DistanceUnit.CM) < distanceThreshold;
    }

    public double getDistance(){
        return distanceSensor.getDistance(DistanceUnit.CM);
    }


}