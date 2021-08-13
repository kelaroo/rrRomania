package org.firstinspires.ftc.teamcode.tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class CameraDebug2 extends LinearOpMode {
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
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);

        webcam.setPipeline(new CameraPipeLine(gamepad1, telemetry));

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        waitForStart();

        while(opModeIsActive())
            ;
    }

    public static class CameraPipeLine extends OpenCvPipeline {
        public enum RingsNumber {
            FOUR,
            ONE,
            NONE
        }

        int cursorX = 0;
        int cursorY = 0;
        int width = 10;
        int height = 10;

        Gamepad gamepad1;
        Telemetry telemetry;

        public CameraPipeLine(Gamepad g1, Telemetry t) {
            gamepad1 = g1;
            telemetry = t;
        }

        static RingsNumber ringsNumber;
        static int nrPixels = -1;

        @Override
        public Mat processFrame(Mat input) {
            if(gamepad1.dpad_left)  cursorX--;
            if(gamepad1.dpad_right) cursorX++;
            if(gamepad1.dpad_down)  cursorY--;
            if(gamepad1.dpad_up)    cursorY++;

            if(gamepad1.x) width--;
            if(gamepad1.b) width++;
            if(gamepad1.a) height--;
            if(gamepad1.y) height++;

            telemetry.addData("Corner", String.format("%d, %d", cursorX, cursorY));
            telemetry.addData("Width", width);
            telemetry.addData("Height", height);
            telemetry.update();

            if(!gamepad1.left_stick_button) {
                Imgproc.rectangle(input, new Point(cursorX, cursorY), new Point(cursorX+width, cursorY+height),
                        new Scalar(0, 255, 0));
                return input;
            } else {

                Rect cropRect = new Rect(cursorX, cursorY, width, height);

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

                if(gamepad1.right_stick_button)
                    return input;
                else
                    return filtered;
            }
        }
    }
}
