package frc.robot.subsystems;


import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavXGyroSubsystem extends SubsystemBase {

    private AHRS ahrs = new AHRS(AHRS.NavXComType.kMXP_SPI);

    private final static NavXGyroSubsystem INSTANCE = new NavXGyroSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static NavXGyroSubsystem getInstance() {
        return INSTANCE;
    }

    private NavXGyroSubsystem() {

    }
}

