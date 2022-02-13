package org.firstinspires.ftc.teamcode.testing;

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

public class DetectorV2 {
    OpenCvCamera phoneCam;
    HardwareMap hwmp;
    Telemetry telemetry;
    String position = "none";

    public static int x1 = 20;
    public static int x2 = 20;
    public static int y1 = 100;
    public int y2 = 100;

    int width = 100;
    int height = 100;

    public DetectorV2(String Color, HardwareMap hw, Telemetry telemetry){
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
        Rect leftRect = new Rect(x1,y1,width,height);
        Rect rightRect = new Rect(x2,y2,width,height);
        Mat leftCrop = new Mat();
        Mat rightCrop = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.rectangle(input,leftRect,new Scalar(200,200,200),2);
            Imgproc.rectangle(input,rightRect,new Scalar(200,200,200),2);

            leftCrop = input.submat(leftRect);
            rightCrop = input.submat(rightRect);

            Core.extractChannel(leftCrop,leftCrop,1);
            Core.extractChannel(rightCrop,rightCrop,1);

            double leftColor = Core.mean(leftCrop).val[0];
            double rightColor = Core.mean(rightCrop).val[0];

            if (leftColor > 111 && leftColor < 70) {
                 position = "left";
            } else if (rightColor > 111 && rightColor < 70) {
                position = "middle";
            } else {
                position = "right";
            }
            return input;
        }
    }
}
