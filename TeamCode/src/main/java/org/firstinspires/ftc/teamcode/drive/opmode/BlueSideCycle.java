package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "Auto")
public class BlueSideCycle extends LinearOpMode {
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
                .addSpatialMarker(new Vector2d(78,-13), () -> {
                    Deposit.state = Deposit.State.START_AUTO;
                })
                .splineTo(new Vector2d(60,-28),Math.toRadians(-90))
                .addSpatialMarker(new Vector2d(60,-28), () -> {
                    Deposit.stateR = Deposit.StateR.RESET_AUTO;
                })
                .setReversed(false)
                .splineTo(new Vector2d(82,-2),Math.toRadians(0))
                .addSpatialMarker(new Vector2d(82,-2), () -> {
                    intake.setPower(-.7);
                })
                .splineTo(new Vector2d(110,-2),Math.toRadians(0))

                .build();

        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(new Pose2d(110,-2,Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(110,-2), () -> {
                    intake.setPower(.7);
                    Deposit.state = Deposit.State.START_AUTO;

                })
                .setReversed(true)
                .splineTo(new Vector2d(82,-2),Math.toRadians(0))


                .build();



        waitForStart();
        drive.followTrajectorySequenceAsync(seq);
        drive.followTrajectorySequenceAsync(seq1);



        while(opModeIsActive()){
            drive.update();
            deposit.updateAuto();
            deposit.resetUpdateAuto();
        }

    }
}
