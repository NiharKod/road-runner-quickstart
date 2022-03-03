package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.Slides;

@TeleOp
public class DepositTest extends LinearOpMode {
    Slides slides;
    public static double extend3Main =  .3;
    public static double extend3Second = .7;
    @Override
    public void runOpMode() throws InterruptedException {
        slides = new Slides(hardwareMap);
        Slides.state = Slides.State.INACTIVE;
        slides.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ServoImplEx armMain = hardwareMap.get(ServoImplEx.class, "armMain");
        ServoImplEx armSecond = hardwareMap.get(ServoImplEx.class,"armSecond");
        ServoImplEx claw = hardwareMap.get(ServoImplEx.class, "bucket");

        armMain.setPosition(extend3Main);
        armSecond.setPosition(extend3Second);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                Slides.state = Slides.State.LIFT;
            }

            if(gamepad1.b){
                Slides.state = Slides.State.RESET;
            }
            slides.liftUpdate();
            telemetry.addData("position",slides.leftMotor.getCurrentPosition());
            telemetry.update();

        }
    }

}
