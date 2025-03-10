package frc.robot.subsystems.DriveTrain.Widget;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.utils.enums.SwerveDrive.SwervePosition;

import java.util.Map;

public class DriveModuleWidget {

    private final String identifier;
    private GenericEntry driveMotorWidget;
    private GenericEntry rotateMotorWidget;

    private GenericEntry encoderWidget;
    private GenericEntry rotationWidget;

    private GenericEntry driveEncoderWidget;

    public DriveModuleWidget(SwervePosition pos) {
        this.identifier = pos.toString();
        this.initialize();
    }

    private void initialize() {
        driveMotorWidget = Shuffleboard.getTab("Swerve " + identifier)
            .add("Drive Motor Output" + identifier, 0.0)
            .withSize(2, 1)
            .withPosition(0, 0)
            // .withWidget(BuiltInWidgets.kMotorController)
            .getEntry();

        rotateMotorWidget = Shuffleboard.getTab("Swerve " + identifier)
            .add("Rotate Motor Output" + identifier, 0.0)
            .withSize(2, 1)
            .withPosition(0, 1)
            // .withWidget(BuiltInWidgets.kMotorController)
            .getEntry();

        encoderWidget = Shuffleboard.getTab("Swerve " + identifier)
            .add("CANCODER Value " + identifier, 0.0)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();
        rotationWidget = Shuffleboard.getTab("Swerve " + identifier)
            .add("Rotation " + identifier, 0)
            .withPosition(2, 1)
            .withWidget(BuiltInWidgets.kGyro)
            .withProperties(Map.of("min", -180, "max", 180))
            .getEntry();

        driveEncoderWidget = Shuffleboard.getTab("Swerve " + identifier)
            .add("DRIVE ENCODER VALUE", 0)
            .getEntry();
    }

    public void setState(SwerveModuleState state) {

    }
}
