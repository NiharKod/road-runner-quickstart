package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.Tele;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class COMPTELEOP extends LinearOpMode {
    Deposit deposit;
    DcMotorEx FL, FR, BL, BR;
    Intake intake;
    CRServo spinner;



    @Override
    public void runOpMode() throws InterruptedException {

        deposit = new Deposit(hardwareMap,gamepad1, new ElapsedTime(), telemetry);
        intake = new Intake(hardwareMap);
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
         spinner = hardwareMap.get(CRServo.class, "spinner");


        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);



        Deposit.state = Deposit.State.IDLE;
        Deposit.stateR = Deposit.StateR.IDLE;
        Slides.state = Slides.State.RESET;

        deposit.wristIntake();
        deposit.armIntake();
        deposit.openClaw();
        waitForStart();

        while(opModeIsActive()){
          //  telemetry.addData("Left Motor Ticks", slides.leftMotor.getCurrentPosition());
          //  telemetry.update();

          //  TelemetryPacket packet = new TelemetryPacket();
          //  packet.put("Position", slides.leftMotor.getCurrentPosition());
           // packet.put("Desired", 200);

          //  FtcDashboard dashboard = FtcDashboard.getInstance();
           // dashboard.sendTelemetryPacket(packet);

            double leftPower = gamepad1.left_stick_y;
            double rightPower = gamepad1.right_stick_y;

            FL.setPower(leftPower);
            BL.setPower(leftPower);
            FR.setPower(rightPower);
            BR.setPower(rightPower);

            intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);


            boolean spinnerLeft = gamepad1.dpad_left;
            boolean spinnerRight = gamepad1.dpad_right;
            if(spinnerLeft){
                spinner.setPower(1);
            }
            if(spinnerRight){
                spinner.setPower(-1);
            }
            if(gamepad1.dpad_down){
                spinner.setPower(0);
            }




            deposit.update();
            deposit.resetUpdate();
        }

    }
}
