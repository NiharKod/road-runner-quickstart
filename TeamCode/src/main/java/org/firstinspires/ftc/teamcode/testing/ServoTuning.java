package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "ServoTuning")

public class ServoTuning extends LinearOpMode {
    public static double resetMain = 1;
    public static double resetSecond = 0;
    public static double extendLowSecond= .85;
    public static double extendLowMain = .15;
    public static double idleMain = .9;
    public static double idleSecond =.1;
    public static double extend3Main =  .3;
    public static double extend3Second = .7;
    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx armMain = hardwareMap.get(ServoImplEx.class, "armMain");
        ServoImplEx armSecond = hardwareMap.get(ServoImplEx.class,"armSecond");
        ServoImplEx claw = hardwareMap.get(ServoImplEx.class, "bucket");

        armMain.setPwmRange(new PwmControl.PwmRange(500,2500));
        armSecond.setPwmRange(new PwmControl.PwmRange(500,2500));

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
               armMain.setPosition(extend3Main);
               armSecond.setPosition(extend3Second);
            }

            if(gamepad1.b){
                armMain.setPosition(resetMain);
                armSecond.setPosition(resetSecond);
            }

            if(gamepad1.x){
                armMain.setPosition(extendLowMain);
                armSecond.setPosition(extendLowSecond);
            }

            if(gamepad1.y){
                armMain.setPosition(idleMain);
                armSecond.setPosition(idleSecond);
            }
        }



    }
}
