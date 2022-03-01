package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.old.Deposit;
@TeleOp
@Config
public class ServoTester extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime timer = new ElapsedTime();
        Deposit depo = new Deposit(hardwareMap,gamepad1, timer, telemetry);

    //    depo.wrist.setPosition(.5);
        waitForStart();
        while(opModeIsActive()){

            if(gamepad1.a){
                depo.wristIdle();
            }
    //  hi nihar
            if(gamepad1.b){
               depo.wristDeposit();
            }

            if(gamepad1.x){
                depo.wristIntake();
            }



        }
    }
}
