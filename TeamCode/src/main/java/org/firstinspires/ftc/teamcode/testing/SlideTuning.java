package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Slides;
@Config
@TeleOp
public class SlideTuning extends LinearOpMode {
    Slides slides;
    public static double kp = .00001;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        slides = new Slides(hardwareMap);

        PIDFController pidfController = new PIDFController(new PIDCoefficients(kp,ki,kd));

        waitForStart();


        while(opModeIsActive()){
            telemetry.addData("Left Motor Ticks", slides.leftMotor.getCurrentPosition());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Position", slides.leftMotor.getCurrentPosition());
            packet.put("Desired", 200);

            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(packet);

         //   slides.setPower(gamepad1.right_stick_y);










        }

    }
}
