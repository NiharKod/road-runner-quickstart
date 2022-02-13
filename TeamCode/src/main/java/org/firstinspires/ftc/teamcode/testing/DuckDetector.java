package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class DuckDetector{
    OpenCvCamera phoneCam;
    Mat HSVmat = new Mat();
    Mat contoursOnFrameMat = new Mat();

    List<MatOfPoint> contoursList = new ArrayList<>();

    int numContoursFound = 0;

    public static Scalar lowerHSV = new Scalar(0,100,25);
    public static Scalar upperHSV = new Scalar(0,100,90);

    public double threshold = 100;

    public double blurConstant = 1;

    public double dilationConstant = 2;

    int duckPosition = 0;

    Telemetry telemetryOpenCV = null;
    HardwareMap hwmp;

    public DuckDetector(HardwareMap hw, Telemetry OpModeTelemetry){
        this.hwmp = hw;
        telemetryOpenCV = OpModeTelemetry;
    }

    public void init(){
        int cameraMonitorViewId = hwmp.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwmp.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.setPipeline(new DuckDetector.DuckDetectingPipeline());
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

    public int getDuckPosition(){
        return duckPosition;
    }

    class DuckDetectingPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            //clear
            contoursList.clear();
            Imgproc.cvtColor(input, HSVmat, Imgproc.COLOR_RGB2HSV_FULL);

            //filters out all the colors but the yellow
            Core.inRange(HSVmat, lowerHSV, upperHSV, HSVmat);

            Size kernelSize = new Size(blurConstant, blurConstant);

            //blurs the image
            Imgproc.GaussianBlur(HSVmat, HSVmat, kernelSize, 0);

            Size kernelSize2 = new Size(2 * dilationConstant + 1, 2 * dilationConstant + 1);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kernelSize2);

            Imgproc.dilate(HSVmat, HSVmat, kernel);

            //finding the contours
            Imgproc.findContours(HSVmat, contoursList, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            numContoursFound = contoursList.size();
            input.copyTo(contoursOnFrameMat);

            for (MatOfPoint contour : contoursList) {
                Rect rect = Imgproc.boundingRect(contour);
                telemetryOpenCV.addData("RectX", rect.x);
                telemetryOpenCV.addData("RectY", rect.y);
                telemetryOpenCV.update();

                if (rect.y >= threshold) {
                    Imgproc.rectangle(contoursOnFrameMat, rect.tl(), rect.br(), new Scalar(255, 0, 0), 2);
                    Imgproc.putText(contoursOnFrameMat, String.valueOf(rect.x), rect.tl(), 0, 0.5, new Scalar(255, 255, 255));

                    if (rect.x >= 100) {
                        duckPosition = 2;
                    } else {
                        duckPosition = 0;
                    }
                }
            }

            return input;
        }
    }
}