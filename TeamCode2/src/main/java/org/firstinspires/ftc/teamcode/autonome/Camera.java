package org.firstinspires.ftc.teamcode.autonome;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Camera {
    public OpenCvCamera webcam;

    static public int H_MIN = 9;    // 9
    static public int H_MAX = 45;   //65
    static public int S_MIN = 60;   //60
    static public int S_MAX = 256;  //256
    static public int V_MIN = 95;   //95
    static public int V_MAX = 256;  //256

    final static private int ERODE_SIZE = 3;
    final static private int DILATE_SIZE = 8;

    public Camera(HardwareMap hw) {
        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hw.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new RingsDetectionPipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 0);
    }

    public static class RingsDetectionPipeline extends OpenCvPipeline {

        public enum RingsNumber {
            FOUR,
            ONE,
            NONE
        }

        static RingsNumber ringsNumber;
        static int nrPixels = -1;

        @Override
        public Mat processFrame(Mat input) {
            Rect cropRect = new Rect(99, 97, 111, 74);

            input = input.submat(cropRect);
            Mat filtered = input.clone();
            Imgproc.cvtColor(filtered, filtered, Imgproc.COLOR_RGB2HSV);

            Core.inRange(filtered, new Scalar(H_MIN, S_MIN, V_MIN), new Scalar(H_MAX, S_MAX, V_MAX), filtered);

            Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(ERODE_SIZE, ERODE_SIZE));
            Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(DILATE_SIZE, DILATE_SIZE));
            Imgproc.erode(filtered, filtered, erodeElement);
            Imgproc.dilate(filtered, filtered, dilateElement);

            erodeElement = null;
            dilateElement = null;

            nrPixels = Core.countNonZero(filtered);
            if(nrPixels >= 2500) {
                ringsNumber = RingsDetectionPipeline.RingsNumber.FOUR;
            } else if(nrPixels >= 900) {
                ringsNumber = RingsDetectionPipeline.RingsNumber.ONE;
            } else {
                ringsNumber = RingsDetectionPipeline.RingsNumber.NONE;
            }

            return filtered;
            //return input;
        }

        public static RingsDetectionPipeline.RingsNumber getNumberOfRings() {return ringsNumber;}
        public int getNrPixels() {return nrPixels;}
    }
}
