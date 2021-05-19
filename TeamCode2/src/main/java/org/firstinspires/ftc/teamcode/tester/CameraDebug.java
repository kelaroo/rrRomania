package org.firstinspires.ftc.teamcode.tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonome.Camera;
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

public class CameraDebug extends LinearOpMode {
    public OpenCvCamera webcam;

    static public int H_MIN = 9;    // 9
    static public int H_MAX = 45;   //65
    static public int S_MIN = 60;   //60
    static public int S_MAX = 256;  //256
    static public int V_MIN = 95;   //95
    static public int V_MAX = 256;  //256

    final static private int ERODE_SIZE = 3;
    final static private int DILATE_SIZE = 8;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new Camera.RingsDetectionPipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 0);
    }

    public static class CameraPipeLine extends OpenCvPipeline {
        public enum RingsNumber {
            FOUR,
            ONE,
            NONE
        }

        static Game

        static RingsNumber ringsNumber;
        static int nrPixels = -1;

        @Override
        public Mat processFrame(Mat input) {
            Rect cropRect = new Rect(105, 35, 100, 100);

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
            if(nrPixels >= 3000) {
                ringsNumber = Camera.RingsDetectionPipeline.RingsNumber.FOUR;
            } else if(nrPixels >= 800) {
                ringsNumber = Camera.RingsDetectionPipeline.RingsNumber.ONE;
            } else {
                ringsNumber = Camera.RingsDetectionPipeline.RingsNumber.NONE;
            }

            return filtered;
            //return input;
        }
    }
}
