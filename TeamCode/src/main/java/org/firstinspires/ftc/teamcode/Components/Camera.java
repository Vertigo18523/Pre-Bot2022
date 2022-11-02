package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Base.Component;
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

public class Camera implements Component {
    private final OpenCvCamera camera;

    private final Telemetry telemetry;

    public Camera(String deviceName, HardwareMap hardwareMap, Telemetry telemetry) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, deviceName), hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        camera.setPipeline(new SleeveDetection());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void start() {}

    @Override
    public void update() {}

    public enum ParkingPosition {
        DEFAULT,
        LEFT,
        CENTER,
        RIGHT
    }

    public ParkingPosition getPosition() {
        return position;
    }

    private static final int
            X = 145,
            Y = 168,
            W = 30,
            H = 50;

    private static final Scalar
            lower_yellow_bounds  = new Scalar(100, 100, 0, 255),
            upper_yellow_bounds  = new Scalar(255, 255, 200, 255),
            lower_cyan_bounds    = new Scalar(0, 100, 100, 255),
            upper_cyan_bounds    = new Scalar(200, 255, 255, 255),
            lower_magenta_bounds = new Scalar(100, 0, 100, 255),
            upper_magenta_bounds = new Scalar(255, 200, 255, 255);

    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255),
            WHITE = new Scalar(255, 255, 255);

    private double
            yelPercent,
            cyaPercent,
            magPercent;

    private final Mat
            yelMat = new Mat(),
            cyaMat = new Mat(),
            magMat = new Mat();

    private Mat
            blurredMat = new Mat(),
            kernel = new Mat();

    private final Point
            sleeve_pointA = new Point(X, Y),
            sleeve_pointB = new Point(X + W,Y + H);

    private volatile ParkingPosition position = ParkingPosition.DEFAULT;

    class SleeveDetection extends OpenCvPipeline {
        public Mat processFrame(Mat input) {
            Imgproc.blur(input, blurredMat, new Size(5, 5));
            blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));

            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
            Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

            Core.inRange(blurredMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);
            Core.inRange(blurredMat, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
            Core.inRange(blurredMat, lower_magenta_bounds, upper_magenta_bounds, magMat);

            yelPercent = Core.countNonZero(yelMat);
            cyaPercent = Core.countNonZero(cyaMat);
            magPercent = Core.countNonZero(magMat);

            double maxPercent = Math.max(yelPercent, Math.max(cyaPercent, magPercent));

            if (maxPercent == yelPercent) {
                position = ParkingPosition.LEFT;
                Imgproc.rectangle(
                        input,
                        sleeve_pointA,
                        sleeve_pointB,
                        YELLOW,
                        2
                );
            } else if (maxPercent == cyaPercent) {
                position = ParkingPosition.CENTER;
                Imgproc.rectangle(
                        input,
                        sleeve_pointA,
                        sleeve_pointB,
                        CYAN,
                        2
                );
            } else if (maxPercent == magPercent) {
                position = ParkingPosition.RIGHT;
                Imgproc.rectangle(
                        input,
                        sleeve_pointA,
                        sleeve_pointB,
                        MAGENTA,
                        2
                );
            } else {
                position = ParkingPosition.DEFAULT;
                Imgproc.rectangle(
                        input,
                        sleeve_pointA,
                        sleeve_pointB,
                        WHITE,
                        2
                );
            }

            blurredMat.release();
            yelMat.release();
            cyaMat.release();
            magMat.release();
            kernel.release();

            return input;
        }
    }
}