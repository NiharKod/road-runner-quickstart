package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class DetectorV2 {
   public OpenCvCamera phoneCam;
    HardwareMap hwmp;
    Telemetry telemetry;
    String position = "none";

    public static int x1 = 10;
    public static int y1 = 185;
    public static int x2 = 10;
    public static int y2 = 410;

    int width = 100;
    int height = 100;

    double middleColor;
    double rightColor;

    public DetectorV2(HardwareMap hw, Telemetry telemetry){
        this.hwmp = hw;
        this.telemetry = telemetry;

    }

    public void init(){
        int cameraMonitorViewId = hwmp.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwmp.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.setPipeline(new DetectorV2.Detecting());
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

   public String getPosition(){
        telemetry.addLine(position); return position;
   }
    class Detecting extends OpenCvPipeline {
        Rect middleRect = new Rect(x1,y1,width,height);
        Rect rightRect = new Rect(x2,y2,width,height);
        Mat middleCrop = new Mat();
        Mat rightCrop = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.rectangle(input,middleRect,new Scalar(200,200,200),2);
            Imgproc.rectangle(input,rightRect,new Scalar(200,200,200),2);

            middleCrop = input.submat(middleRect);
            rightCrop = input.submat(rightRect);

            Core.extractChannel(middleCrop,middleCrop,1);
            Core.extractChannel(rightCrop,rightCrop,1);

             middleColor = Core.mean(middleCrop).val[0];
             rightColor = Core.mean(rightCrop).val[0];

             double value = middleColor - rightColor;
             if(value > 20){
                 position = "right";
             }else if(value < -20){
                 position = "middle";
             }else {
                 position = "left";
             }
            middleCrop.release();
            rightCrop.release();
            return input;
        }

    }

    public double getMiddleColor(){
        return middleColor;
    }

    public double getRightColor(){
        return rightColor;
    }
}
