package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
@Disabled
@Autonomous
public class BlueAsync extends OpMode {
    SampleTankDrive drive;
    Intake intake;
    Deposit deposit;
    @Override
    public void init() {

        intake = new Intake(hardwareMap);
        drive = new SampleTankDrive(hardwareMap);
        deposit = new Deposit(hardwareMap,gamepad1, new ElapsedTime(), telemetry);
        drive.setPoseEstimate(new Pose2d(30,-7,Math.toRadians(-90)));


        Deposit.state = Deposit.State.IDLE;
        Deposit.stateR = Deposit.StateR.IDLE;
        Slides.state = Slides.State.RESET;

        deposit.wristIntake();
        deposit.armIntake();
        deposit.openClaw();

       Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    Deposit.state = Deposit.State.START_AUTO;
                })
                .splineTo(new Vector2d(44, -26), Math.toRadians(-45))
                .build();

       Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(13, -13), Math.toRadians(180))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(traj1))
                .build();

         drive.followTrajectoryAsync(traj);
        drive.followTrajectoryAsync(traj1);


    }

    @Override
    public void loop() {
        drive.update();
      //  deposit.updateAuto();
    }
}
