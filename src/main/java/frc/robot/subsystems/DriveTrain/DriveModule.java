package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.utils.enums.SwerveDrive.SwervePosition;
import com.revrobotics.spark.SparkMax;

//スワーブの車輪一個用のクラス
public class DriveModule {

    //機器類
    private final SparkMax driveMotor;
    private final SparkMax steeringMotor;
    private final CANcoder encoder;

    private SwerveModuleState currentState;

    //初期化
    public DriveModule(SwervePosition pos,
                       int driveMotorPort,
                       int steeringMotorPort,
                       int encoderPort
    ) {

        this.driveMotor = new SparkMax(driveMotorPort, SparkLowLevel.MotorType.kBrushless);
        this.steeringMotor = new SparkMax(steeringMotorPort, SparkLowLevel.MotorType.kBrushless);
        this.encoder = new CANcoder(encoderPort);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getAbsoluteEncoder().getPosition() / Constants.SwerveConstants.gearRatio
            * Constants.SwerveConstants.wheelRaduis * 2 * Math.PI/* add gear ratio calc */,
            Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue().in(Units.Rotations))
            );
    }


    public void setState(SwerveModuleState state){
        state.optimize(currentState.angle);
        driveMotor.set(state.speedMetersPerSecond);
        //driveMotor.set(state.angle);
    }
}
