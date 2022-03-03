package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

@Autonomous
public class BluSideMEC extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        drive.setPoseEstimate(new Pose2d(78,-7,Math.toRadians(0)));

        waitForStart();
        TrajectorySequence seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(78,-4))
                .turn(Math.toRadians(100), 70,65)
                .build();
        drive.followTrajectorySequenceAsync(seq);

        while(opModeIsActive()){
            drive.update();
        }
    }

}
