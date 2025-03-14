package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain.Widget.DriveModuleWidget;
import frc.robot.utils.enums.SwerveDrive.SwervePosition;
import com.revrobotics.spark.SparkMax;

import java.util.Optional;

//スワーブの車輪一個用のクラス
public class DriveModule {

    //機器類
    private final SparkMax driveMotor;
    private final SparkMax steeringMotor;
    private final CANcoder encoder;
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder steeringEncoder;
    private final SparkClosedLoopController driveCloopController;
    private final SparkClosedLoopController steeringCloopController;

    //車輪角度のオフセット
    private final Rotation2d offsetAngle;

    private SwerveModuleState currentState;

    private final DriveModuleWidget widget;

    //初期化
    public DriveModule(SwervePosition pos,
                       int driveMotorPort,
                       int steeringMotorPort,
                       int encoderPort,
                       Rotation2d offset
    ) {

        this.driveMotor = new SparkMax(driveMotorPort, SparkLowLevel.MotorType.kBrushless);
        this.steeringMotor = new SparkMax(steeringMotorPort, SparkLowLevel.MotorType.kBrushless);
        this.encoder = new CANcoder(encoderPort);
        this.driveEncoder = this.driveMotor.getEncoder();
        this.steeringEncoder = this.steeringMotor.getAbsoluteEncoder();
        this.driveCloopController = this.driveMotor.getClosedLoopController();
        this.steeringCloopController = this.steeringMotor.getClosedLoopController();
        this.offsetAngle = offset;
        this.widget = new DriveModuleWidget(pos);
        this.driveEncoder.setPosition(this.encoder.getAbsolutePosition().getValue().in(Units.Radian));
        this.driveMotor.configure(Config.drivingConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        this.steeringMotor.configure(Config.steeringConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    //エンコーダーの値を取得してSwerveModulePositionとして返す
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getAbsoluteEncoder().getPosition() / Constants.SwerveConstants.gearRatio
            * Constants.SwerveConstants.wheelRaduis * 2 * Math.PI/* add gear ratio calc */,
            Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue().in(Units.Rotations))
            );
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue().in(Units.Rotations))
        );
    }


    public void setState(SwerveModuleState state){
        //角度の最適化(角度差π以上の場合π以下になるように調整)
        state.optimize(Rotation2d.fromRotations(encoder.getPosition().getValue().in(Units.Rotations)));
        //閉ループ制御で目標値をセット(出力はモータードライバーが調整
        driveCloopController.setReference(state.speedMetersPerSecond, SparkBase.ControlType.kVelocity);
        steeringCloopController.setReference(state.angle.getRadians(), SparkBase.ControlType.kPosition);
        currentState = state;
        this.setWidget();

        //driveMotor.set(state.angle);
    }

    private void setWidget(){
        widget.setMotorState(driveMotor.getAppliedOutput(),steeringMotor.getAppliedOutput());
        widget.setEncoderValue(encoder.getAbsolutePosition().getValue().in(Units.Degree));
        widget.setSteeringEncoderWidget(steeringEncoder.getPosition());
    }
}
