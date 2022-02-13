package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DuckDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
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
    String ringPosition = "none";
   // double rows, rect1Cols, rect2Cols;

    double lowColor;
    double upColor;

    double rows;
    double rows2;
    double rect1Cols;
    double rect2Cols;

    public DetectorV2(String Color, HardwareMap hw, Telemetry telemetry, double rows, double rows2, double rect1Cols, double rect2Cols){
        this.hwmp = hw;
        this.telemetry = telemetry;
        this.rows = rows;
        this.rows2 = rows2;
        this.rect1Cols = rect1Cols;
        this.rect2Cols = rect2Cols;
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

    public String getRingPosition(){
        return ringPosition; }


    class Detecting extends OpenCvPipeline {


        Mat YCbCr = new Mat();
        Mat upperCrop = new Mat();
        Mat lowerCrop = new Mat();
        Mat outputMat = new Mat();


        @Override
        public Mat processFrame(Mat input) {
            //convert mat.
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            //copy input mat onto output
            input.copyTo(outputMat);


            Rect rect = new Rect((int) Math.round(YCbCr.rows() * rows), (int) Math.round(YCbCr.cols() * rect1Cols), 100, 69);

            Rect rect2 = new Rect((int) Math.round(YCbCr.rows() * rows), (int) Math.round(YCbCr.cols() * rect2Cols), 100, 20);


            Imgproc.rectangle(outputMat, rect, new Scalar(0, 0, 255), 2);
            Imgproc.rectangle(outputMat, rect2, new Scalar(0, 0, 255), 2);


            //crop the top part of the stack, and the bottom part of the stack.
            lowerCrop = YCbCr.submat(rect2);
            upperCrop = YCbCr.submat(rect);


            //extract the Cb channel
            Core.extractChannel(lowerCrop, lowerCrop, 2);
            Core.extractChannel(upperCrop, upperCrop, 2);


            //take average
            Scalar lowerColor = Core.mean(lowerCrop);
            Scalar upperColor = Core.mean(upperCrop);

            lowColor = lowerColor.val[0];
            upColor = upperColor.val[0];

            //check which one to determine the

            if (lowColor > 111 && upColor > 111 && lowColor < 120 && upColor < 120) {
                ringPosition = "left";
            } else if (lowColor > 135 && upColor > 125) {
                ringPosition = "middle";
            } else {
                ringPosition = "right";
            }

            return outputMat;
        }
    }
    public double getVal(){
        return lowColor;
    }

    public double getVal2(){
        return upColor;
    }
}
