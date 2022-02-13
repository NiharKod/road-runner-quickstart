package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class CV extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DetectorV2 detector = new DetectorV2(hardwareMap, telemetry);
        detector.init();
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        while(!isStarted()) {
            telemetry.addData("MiddleColor", detector.getMiddleColor());
            telemetry.addData("RightColor", detector.getRightColor());
            telemetry.update();
            packet.put("Middle",detector.getMiddleColor());
            packet.put("Right",detector.getRightColor());
            packet.put("Position", detector.getPosition());
            dashboard.sendTelemetryPacket(packet);
            FtcDashboard.getInstance().startCameraStream(detector.phoneCam, 4);
        }
        waitForStart();
        while (opModeIsActive()){

        }
    }
}
