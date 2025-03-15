package frc.robot.subsystems;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class CameraSubsystem extends SubsystemBase {

    private final static CameraSubsystem INSTANCE = new CameraSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static CameraSubsystem getInstance() {
        return INSTANCE;
    }

    private Thread visionThread;

    private CameraSubsystem() {
        visionThread = new Thread(
            () -> {
                UsbCamera camera = CameraServer.startAutomaticCapture();
                camera.setResolution(640, 480);

                CvSink cvSink = CameraServer.getVideo();
                CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

                Mat mat = new Mat();

                while (!Thread.interrupted()) {

                    if (cvSink.grabFrame(mat) == 0) {
                        outputStream.notifyError(cvSink.getError());
                        continue;
                    }
                    Imgproc.rectangle(
                        mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                    outputStream.putFrame(mat);
                }
            });
    }
    public void start(){
        visionThread.setDaemon(true);
        visionThread.start();
    }
}

