package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DuckDetector;

@Autonomous
public class CV extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DuckDetector detector = new DuckDetector(hardwareMap, telemetry);
        detector.init();
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        waitForStart();
        while (opModeIsActive()){

        }
    }
}
