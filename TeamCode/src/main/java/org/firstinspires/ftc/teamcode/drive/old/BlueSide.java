package org.firstinspires.ftc.teamcode.drive.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.subsystems.old.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.old.Intake;
import org.firstinspires.ftc.teamcode.subsystems.old.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "Auto")
public class BlueSide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Deposit.state = Deposit.State.IDLE;
        Deposit.stateR = Deposit.StateR.IDLE;
        Slides.state = Slides.State.RESET;
        CRServo spinner = hardwareMap.get(CRServo.class, "spinner");
        Deposit deposit = new Deposit(hardwareMap,gamepad1, new ElapsedTime(), telemetry);

        drive.setPoseEstimate(new Pose2d(30,-7,Math.toRadians(-90)));

        Deposit.state = Deposit.State.IDLE;
        Deposit.stateR = Deposit.StateR.IDLE;
        Slides.state = Slides.State.RESET;

        deposit.wristIntake();
        deposit.armIntake();
        deposit.closeClaw();

        TrajectorySequence seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(10, -9.5), Math.toRadians(180))
                .addTemporalMarker(() -> {
                    spinner.setPower(1);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    spinner.setPower(0);
                })
                .setReversed(true)
                .addDisplacementMarker(() -> {
                    Deposit.state = Deposit.State.START_AUTO;
                })
                .splineTo(new Vector2d(46.5, -35.5), Math.toRadians(-45))
                .addDisplacementMarker(() -> {
                    Deposit.stateR = Deposit.StateR.RESET_AUTO;
                })
                .setReversed(false)
                .splineTo(new Vector2d(12, -36), Math.toRadians(180))
                .build();
        drive.followTrajectorySequenceAsync(seq);

        waitForStart();


      while(opModeIsActive()){
            drive.update();
          //  deposit.updateAuto();
           // deposit.resetUpdateAuto();
      }

    }
}
