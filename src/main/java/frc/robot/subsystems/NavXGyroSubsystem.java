package frc.robot.subsystems;


import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavXGyroSubsystem extends SubsystemBase {

    //シングルトン
    private final static NavXGyroSubsystem INSTANCE = new NavXGyroSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static NavXGyroSubsystem getInstance() {
        return INSTANCE;
    }

    private final AHRS ahrs = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private final DoublePublisher publisher;

    //初期化
    private NavXGyroSubsystem() {
        //角度をリセット
        ahrs.zeroYaw();
        //ダッシュボードに表示するウィジェット
        publisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("NavX Gyro Rotation (radians)").publish();
    }


    public AHRS getSensor (){
        return this.ahrs;
    }

    //
    @Override
    public void periodic(){
        //角度を更新
        publisher.set(Rotation2d.fromDegrees(ahrs.getAngle()).getRadians());
    }
}

