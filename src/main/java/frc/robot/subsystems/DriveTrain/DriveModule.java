package frc.robot.subsystems.DriveTrain;

import com.revrobotics.spark.SparkLowLevel;
import frc.robot.utils.enums.SwerveDrive.SwerveModuleCANCoderPort;
import frc.robot.utils.enums.SwerveDrive.SwerveModulePosition;
import com.revrobotics.spark.SparkMax;
import frc.robot.utils.enums.SwerveDrive.SwerveModuleRunningMotorPort;
import frc.robot.utils.enums.SwerveDrive.SwerveModuleSteeringMotorPort;

public class DriveModule {

    private final SparkMax driveMotor;

    public DriveModule(SwerveModulePosition pos,
                       SwerveModuleRunningMotorPort driveMotorPort,
                       SwerveModuleSteeringMotorPort steeringMotorPort,
                       SwerveModuleCANCoderPort encoderPort
                       ){

        this.driveMotor = new SparkMax(driveMotorPort, SparkLowLevel.MotorType.kBrushless);

    }
}
