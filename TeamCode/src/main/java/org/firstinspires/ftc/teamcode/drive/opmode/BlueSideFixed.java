package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class BlueSideFixed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Deposit.state = Deposit.State.IDLE;
        Deposit.stateR = Deposit.StateR.IDLE;
        Slides.state = Slides.State.RESET;
        Deposit deposit = new Deposit(hardwareMap,gamepad1, new ElapsedTime(), telemetry);

        drive.setPoseEstimate(new Pose2d(78,-7,Math.toRadians(-90)));

        Deposit.state = Deposit.State.IDLE;
        Deposit.stateR = Deposit.StateR.IDLE;
        Slides.state = Slides.State.RESET;

        deposit.wristIntake();
        deposit.armIntake();
        deposit.closeClaw();

        TrajectorySequence seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(6)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    Deposit.state = Deposit.State.START_AUTO;
                })
                .splineTo(new Vector2d(62,-25),Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Deposit.stateR = Deposit.StateR.RESET_AUTO;
                })
                .setReversed(false)
                .splineTo(new Vector2d(82,-2),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset (0,() -> {
                    intake.setPower(-.7);
                })
                .splineTo(new Vector2d(110,-2),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    intake.setPower(.7);
                    Deposit.state = Deposit.State.START_AUTO;

                })
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    intake.setPower(0);
                })
                .back(28)
                .setReversed(true)
//                .splineTo(new Vector2d(70,-10),Math.toRadians(-30))
//                .setReversed(true)
                .splineTo(new Vector2d(62,-25),Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    Deposit.stateR = Deposit.StateR.RESET_AUTO;
                })
                .setReversed(false)
                .splineTo(new Vector2d(82,0),Math.toRadians(0))

                .splineTo(new Vector2d(110,-2),Math.toRadians(0))


                .build();
        waitForStart();
        drive.followTrajectorySequenceAsync(seq);

        while(opModeIsActive()) {
            drive.update();
            deposit.updateAuto();
            deposit.resetUpdateAuto();
        }
    }
}
