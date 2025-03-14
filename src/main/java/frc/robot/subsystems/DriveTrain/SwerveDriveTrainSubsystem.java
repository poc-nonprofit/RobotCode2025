package frc.robot.subsystems.DriveTrain;

import com.studica.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain.Widget.DriveTrainWidget;
import frc.robot.subsystems.NavXGyroSubsystem;
import frc.robot.utils.enums.SwerveDrive.SwervePosition;


//スワーブ全体を管理するクラス
public class SwerveDriveTrainSubsystem extends SubsystemBase {

    //シングルトンにする
    private final static SwerveDriveTrainSubsystem INSTANCE = new SwerveDriveTrainSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static SwerveDriveTrainSubsystem getInstance() {
        return INSTANCE;
    }

    private final XboxController controller;
    private boolean inputLock = false;
    private final AHRS gyro = NavXGyroSubsystem.getInstance().getSensor();
    //値の急激な変化を避けるため,レートリミッターで変化量を抑制

    private final SlewRateLimiter inputXLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter inputYLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter inputRotLimiter = new SlewRateLimiter(3);
    private double movementSpeed = Constants.SwerveConstants.defaultSpeed;
    private double rotateSpeed = Constants.SwerveConstants.defaultAngularSpeed;
    private boolean isFieldRelative = false;

    //AdvantageScope用Swerve Module Stateパブリッシャー
    private StructArrayPublisher<SwerveModuleState> swerveDriveTrainPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("DriveTrain State", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> swerveModuleStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Swerve Module State", SwerveModuleState.struct).publish();

    //ダッシュボードUI
    private final DriveTrainWidget widget = DriveTrainWidget.getInstance();

    private final DriveModule swerveFR = new DriveModule(
        SwervePosition.FRONT_RIGHT,
        Constants.DriveMotorPort.FRONT_RIGHT,
        Constants.SteeringMotorPort.FRONT_RIGHT,
        Constants.CANCoderPort.FRONT_RIGHT,
        Constants.SwerveConstants.ModuleOffset.FR
    );
    private final DriveModule swerveFL = new DriveModule(
        SwervePosition.FRONT_LEFT,
        Constants.DriveMotorPort.FRONT_LEFT,
        Constants.SteeringMotorPort.FRONT_LEFT,
        Constants.CANCoderPort.FRONT_LEFT,
        Constants.SwerveConstants.ModuleOffset.FL
    );
    private final DriveModule swerveRL = new DriveModule(
        SwervePosition.REAR_LEFT,
        Constants.DriveMotorPort.REAR_LEFT,
        Constants.SteeringMotorPort.REAR_LEFT,
        Constants.CANCoderPort.REAR_LEFT,
        Constants.SwerveConstants.ModuleOffset.RL
    );
    private final DriveModule swerveRR = new DriveModule(
        SwervePosition.REAR_RIGHT,
        Constants.DriveMotorPort.REAR_RIGHT,
        Constants.SteeringMotorPort.REAR_RIGHT,
        Constants.CANCoderPort.REAR_RIGHT,
        Constants.SwerveConstants.ModuleOffset.RR
    );

    private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            Constants.moduleLocations.FL.getValue(),
            Constants.moduleLocations.FR.getValue(),
            Constants.moduleLocations.RL.getValue(),
            Constants.moduleLocations.RR.getValue()
        );

    private final SwerveDriveOdometry odometry =
        new SwerveDriveOdometry(
            kinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[]{
                swerveFL.getPosition(),
                swerveFR.getPosition(),
                swerveRL.getPosition(),
                swerveRR.getPosition(),
            }
        );


    //初期化
    public SwerveDriveTrainSubsystem() {
        this.controller = new XboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
    }

    //モジュールの現在の位置から,オドメトリを更新する.
    private void updateOdometry() {
        this.odometry.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[]{
                swerveFL.getPosition(),
                swerveFR.getPosition(),
                swerveRL.getPosition(),
                swerveRR.getPosition(),
            }
        );
    }

    private void setState(SwerveModuleState[] state) {
        swerveFL.setState(state[0]);
        swerveFR.setState(state[1]);
        swerveRL.setState(state[2]);
        swerveRR.setState(state[3]);
    }

    public void drive(double xInput, double yInput, double rotInput) {
        if (inputLock) return;

        //ロボットの移動方向とスティックの座標系が逆なのでマイナスをかけて,速度の倍率をかける
        double x = -this.movementSpeed * inputXLimiter.calculate(MathUtil.applyDeadband(xInput, Constants.OperatorConstants.CONTROLLER_LEFT_DEADZONE_Y));
        double y = -this.movementSpeed * inputYLimiter.calculate(MathUtil.applyDeadband(yInput, Constants.OperatorConstants.CONTROLLER_LEFT_DEADZONE_X));
        double rot = -this.rotateSpeed * inputRotLimiter.calculate(MathUtil.applyDeadband(rotInput, Constants.OperatorConstants.CONTROLLER_RIGHT_DEADZONE_X));

        //1フレームあたりの移動量を取得
        SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                widget.getIsFieldRelative() ?
                    ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, this.gyro.getRotation2d())
                    : new ChassisSpeeds(x, y, rot),
                Constants.SwerveConstants.defaultPeriod //フレーム時間(0.02s, 50Hz)で移動速度を微分する.
            )
        );
        //最大速度を超えた分は,速度に収まるようにする
        SwerveDriveKinematics.desaturateWheelSpeeds(states, this.movementSpeed);
        this.setState(states);
        //ウィジェットの更新
        this.widget.setInputValue(xInput, yInput, rotInput);
        this.widget.setOutputValue(x, y, rot);
        swerveDriveTrainPublisher.set(states);
    }

    private Command changeSpeed(boolean increase) {
        return runOnce(() -> {
            double spd = this.widget.getMovementSpeed() + (increase ? 0.1 : -0.1);
            this.movementSpeed =
                MathUtil.clamp(spd,
                    0, Constants.SwerveConstants.maxMovementSpeed
                );
            this.widget.setMovementSpeed(spd);

        });
    }

    private Command changeRotateSpeed(boolean increase) {
        return runOnce(() -> {
            double rotateSpd = this.widget.getRotationSpeed() + (increase ? 0.1 : -0.1);
            this.rotateSpeed =
                MathUtil.clamp(rotateSpd,
                    0, Constants.SwerveConstants.maxAngularSpeed
                );
            this.widget.setRotationSpeed(rotateSpd);
        });
    }

    //毎フレーム実行(50Hz)
    @Override
    public void periodic() {
        this.updateOdometry();
        this.widget.setGyroValue(this.gyro.getAngle());
        this.swerveModuleStatePublisher.set(
            new SwerveModuleState[]{
                swerveFL.getState(),
                swerveFR.getState(),
                swerveRL.getState(),
                swerveRR.getState()
            }
        );
    }

    public void setControllerBindings(CommandXboxController controller) {
        //コントローラーの矢印キーで速度関連の調整ができるようにイベントを登録
        controller.povUp().onTrue(this.changeSpeed(true));
        controller.povDown().onTrue(this.changeSpeed(false));
        controller.povLeft().onTrue(this.changeRotateSpeed(true));
        controller.povRight().onTrue(this.changeRotateSpeed(false));
    }
}

