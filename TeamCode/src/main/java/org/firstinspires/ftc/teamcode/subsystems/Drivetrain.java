package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@Config
public class Drivetrain {
    DcMotorEx FL, FR, BR, BL = null;
    SampleTankDrive dt;
    PIDController tController = new PIDController(kp,ki,kd);
    PIDController pController = new PIDController(kp,ki,kd);

    public static double kp = 0.00425;
    public static double ki = 0.11;
    public static double kd = 0;

    public Drivetrain(HardwareMap hardwaremap){
        FL = hardwaremap.get(DcMotorEx.class, "FL");
        FR = hardwaremap.get(DcMotorEx.class, "FR");
        BR = hardwaremap.get(DcMotorEx.class, "BR");
        BL = hardwaremap.get(DcMotorEx.class, "BL");
        dt= new SampleTankDrive(hardwaremap);

    }

    public Pose2d getPosition(){
        return dt.getPoseEstimate();
    }

    public void goToPoint(Pose2d pose, double movementSpeed){
        Pose2d currentPose = getPosition();
        double targetAngle = Math.atan2(pose.getY() - currentPose.getY(), pose.getX()
                - currentPose.getX()) - Math.toRadians(90);
        double turnPower = controller.calculate(currentPose.getHeading(), Math.toDegrees(pose.getHeading()));
    }

    public void update(){
        dt.update();
    }
}
