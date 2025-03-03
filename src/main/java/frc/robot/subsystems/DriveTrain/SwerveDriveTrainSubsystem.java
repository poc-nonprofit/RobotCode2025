package frc.robot.subsystems.DriveTrain;

import com.studica.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
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
    private final SlewRateLimiter inputXLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter inputYLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter inputRotLimiter = new SlewRateLimiter(3);
    private double movementSpeed = Constants.SwerveConstants.defaultMaxSpeed;
    private double rotateSpeed = Constants.SwerveConstants.defaultAngularSpeed;
    private boolean isFieldRelative = false;

    private final DriveModule swerveFR = new DriveModule(
        SwervePosition.FRONT_RIGHT,
        Constants.DriveMotorPort.FRONT_RIGHT,
        Constants.SteeringMotorPort.FRONT_RIGHT,
        Constants.CANCoderPort.FRONT_RIGHT
    );
    private final DriveModule swerveFL = new DriveModule(
        SwervePosition.FRONT_LEFT,
        Constants.DriveMotorPort.FRONT_LEFT,
        Constants.SteeringMotorPort.FRONT_LEFT,
        Constants.CANCoderPort.FRONT_LEFT
    );
    private final DriveModule swerveRL = new DriveModule(
        SwervePosition.REAR_LEFT,
        Constants.DriveMotorPort.REAR_LEFT,
        Constants.SteeringMotorPort.REAR_LEFT,
        Constants.CANCoderPort.REAR_LEFT
    );
    private final DriveModule swerveRR = new DriveModule(
        SwervePosition.REAR_RIGHT,
        Constants.DriveMotorPort.REAR_RIGHT,
        Constants.SteeringMotorPort.REAR_RIGHT,
        Constants.CANCoderPort.REAR_RIGHT
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

    private void drive(double x, double y, double rot) {
        //1フレームあたりの移動量を取得
        SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                this.isFieldRelative ?
                    ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, this.gyro.getRotation2d())
                    : new ChassisSpeeds(x, y, rot),
                Constants.SwerveConstants.defaultPeriod //フレーム時間(0.02s, 50Hz)で移動速度を微分する.
            )
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(states, this.movementSpeed);
    }

    private Command changeSpeed(boolean increase) {
        return runOnce(()->{
            this.movementSpeed += increase ? 0.1:-0.1;
            this.movementSpeed = MathUtil.clamp(this.movementSpeed,0,1);

        });
    }
    private Command changeRotateSpeed(boolean increase) {
        return runOnce(()->{
            this.rotateSpeed += increase ? 0.05:-0.05;
            this.rotateSpeed = MathUtil.clamp(this.rotateSpeed,0,Constants.SwerveConstants.maxAngularSpeed);
        });
    }

    @Override
    public void periodic() {
        if (inputLock) return;

        double x = inputXLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), Constants.OperatorConstants.CONTROLLER_LEFT_DEADZONE_Y));
        double y = inputYLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), Constants.OperatorConstants.CONTROLLER_LEFT_DEADZONE_X));
        double rot = inputRotLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(),Constants.OperatorConstants.CONTROLLER_RIGHT_DEADZONE_X));

        double speedX = x * this.movementSpeed;
        double speedY = y * this.movementSpeed;
        double speedRot = rot * this.rotateSpeed;

        updateOdometry();
        this.drive(speedX, speedY, speedRot);
    }

    public void setControllerBindings(CommandXboxController controller){
        //コントローラーの矢印キーで速度関連の調整ができるようにイベントを登録
        controller.povUp().onTrue(this.changeSpeed(true));
        controller.povDown().onTrue(this.changeSpeed(false));
        controller.povLeft().onTrue(this.changeRotateSpeed(true));
        controller.povRight().onTrue(this.changeRotateSpeed(false));
    }
}

