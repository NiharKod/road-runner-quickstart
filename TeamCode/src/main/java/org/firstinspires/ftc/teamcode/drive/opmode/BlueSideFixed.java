package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.testing.DetectorV2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class BlueSideFixed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Deposit deposit = new Deposit(hardwareMap,gamepad1, new ElapsedTime(), telemetry);

        //set all of the static states to reset;
        Deposit.state = Deposit.State.IDLE;
        Deposit.stateR = Deposit.StateR.IDLE;
        Slides.state = Slides.State.RESET;

        deposit.wristIntake();
        deposit.armIntake();
        deposit.closeClaw();

        DetectorV2 detector = new DetectorV2(hardwareMap, telemetry);
        String position = "";
        detector.init();
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        drive.setPoseEstimate(new Pose2d(78,-7,Math.toRadians(-90)));

        while(!isStarted()) {
            //store the position;
            position = detector.getPosition();
            //telemetry actions
            telemetry.addData("MiddleColor", detector.getMiddleColor());
            telemetry.addData("RightColor", detector.getRightColor());
            telemetry.update();
            packet.put("Middle",detector.getMiddleColor());
            packet.put("Right",detector.getRightColor());
            packet.put("Position", detector.getPosition());
            packet.put("Level", deposit.armLevelThree);
            dashboard.sendTelemetryPacket(packet);
            FtcDashboard.getInstance().startCameraStream(detector.phoneCam, 4);
        }
        //check the position
        if(position == "middle"){
            Deposit.armLevelThree = Deposit.armLevelTwo;
            Slides.setPoint = 0;
        }else if(position =="right"){
            Deposit.armLevelThree = .64;
            Slides.setPoint = 230;
        }else if(position == "left"){
            Deposit.armLevelThree = Deposit.armLevelOne;
            Slides.setPoint = 0;
        }
        waitForStart();
        //generate trajectory
        TrajectorySequence seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(6)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                    Deposit.state = Deposit.State.START_AUTO;
                })
                .splineTo(new Vector2d(62,-23),Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Deposit.stateR = Deposit.StateR.RESET_AUTO;

                })
                .setReversed(false)
                .splineTo(new Vector2d(82,-2),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset (0,() -> {
                    Deposit.armLevelThree = .64;
                    Slides.setPoint = 230;
                    intake.setPower(-.7);
                })
                .splineTo(new Vector2d(112,-2),Math.toRadians(0))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    Deposit.state = Deposit.State.START_AUTO;

                })
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    intake.setPower(.3);
                })
                .back(30)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    intake.setPower(0);
                })
                .setReversed(true)
                .splineTo(new Vector2d(62,-25),Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    Deposit.stateR = Deposit.StateR.RESET_AUTO;
                })
                .setReversed(false)
                .splineTo(new Vector2d(82,0),Math.toRadians(0))
                .splineTo(new Vector2d(110,-2),Math.toRadians(0))
                .build();

        //follow trajectory;
        drive.followTrajectorySequenceAsync(seq);

        while(opModeIsActive()) {
            drive.update();
            deposit.updateAuto(position);
            deposit.resetUpdateAuto(position);
        }
    }
}
