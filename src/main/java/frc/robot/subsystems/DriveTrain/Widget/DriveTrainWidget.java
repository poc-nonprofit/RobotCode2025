package frc.robot.subsystems.DriveTrain.Widget;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain.SwerveDriveTrainSubsystem;

import java.util.Map;

public class DriveTrainWidget {

    private final GenericEntry inputX;
    private final GenericEntry inputY;
    private final GenericEntry inputRotation;
    private final GenericEntry outputX;
    private final GenericEntry outputY;
    private final GenericEntry outputRotation;
    private final GenericEntry gyroValue;
    private final GenericEntry movementSpeed;
    private final GenericEntry rotationSpeed;
    private final GenericEntry isFieldRelative;

    //シングルトンにする
    private final static DriveTrainWidget INSTANCE = new DriveTrainWidget();

    @SuppressWarnings("WeakerAccess")
    public static DriveTrainWidget getInstance() {
        return INSTANCE;
    }

    //ウィジェット類の初期化
    public DriveTrainWidget() {
        System.out.println("Initializing DriveTrainWidget");
        //入力値ウィジェットの初期化
        //Operator Console Dashboardのウィジェット
        ShuffleboardLayout inputContainer = Shuffleboard.getTab("Swerve DriveTrain")
            .getLayout("DriveTrain Input", BuiltInLayouts.kList)
            .withSize(2, 3)
            .withPosition(3, 0);
        this.inputX = inputContainer.add("Input X", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -1, "max", 1)) //値の範囲を-1から1に制限する
            .getEntry();
        this.inputY = inputContainer.add("Input Y", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();
        this.inputRotation = inputContainer.add("Input Rotation", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -1, "max", 1))
            .getEntry();

        //出力値ウィジェットの初期化 (最大速度やリミッターを考慮した値で，実際のモーター出力ではない)
        ShuffleboardLayout outputContainer = Shuffleboard.getTab("Swerve DriveTrain")
            .getLayout("DriveTrain Output", BuiltInLayouts.kList)
            .withSize(2, 3)
            .withPosition(5, 0);
        this.outputX = outputContainer.add("Output X", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -Constants.SwerveConstants.maxMovementSpeed, "max", Constants.SwerveConstants.maxMovementSpeed))
            .getEntry();
        this.outputY = outputContainer.add("Output Y", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -Constants.SwerveConstants.maxMovementSpeed, "max", Constants.SwerveConstants.maxMovementSpeed))
            .getEntry();
        this.outputRotation = outputContainer.add("Output Rotation", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -Constants.SwerveConstants.maxAngularSpeed, "max", Constants.SwerveConstants.maxAngularSpeed))
            .getEntry();

        this.gyroValue = Shuffleboard.getTab("Swerve DriveTrain")
            .add("Chassis Angle", 0)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(7,0)
            .getEntry();

        //最大値等入力を受け付けるウィジェット(ShuffleBoard上で値の変更が可能)
        this.movementSpeed = Shuffleboard.getTab("Swerve DriveTrain")
            .add("Max Speed(m/s)", Constants.SwerveConstants.defaultSpeed)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", 0, "max", Constants.SwerveConstants.maxMovementSpeed))
            .withPosition(0,1)
            .withSize(2, 1)
            .getEntry();
        this.rotationSpeed = Shuffleboard.getTab("Swerve DriveTrain")
            .add("Max Angular Speed(radian/s)",Constants.SwerveConstants.defaultAngularSpeed)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min",0,"max",Constants.SwerveConstants.maxAngularSpeed))
            .withPosition(0,2)
            .withSize(2, 1)
            .getEntry();
        this.isFieldRelative = Shuffleboard.getTab("Swerve DriveTrain")
            .add("Field Oriented Drive", Constants.SwerveConstants.defaultFieldOriented)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(0,0)
            .getEntry();
    }

    //ウィジェットに値をセットする
    public void setInputValue(double x, double y,double rotation) {
        this.inputX.setDouble(x);
        this.inputY.setDouble(y);
        this.inputRotation.setDouble(rotation);
    }
    public void setOutputValue(double x, double y,double rotation) {
        this.outputX.setDouble(x);
        this.outputY.setDouble(y);
        this.outputRotation.setDouble(rotation);
    }

    public void setGyroValue(double gyro) {
        this.gyroValue.setDouble(gyro);
    }
    //入力受付ウィジェット用のゲッター (値がnullの場合はデフォルト値として引数の値が返る)
    public double getMovementSpeed() {
        return this.movementSpeed.getDouble(Constants.SwerveConstants.defaultSpeed);
    }
    public double getRotationSpeed() {
        return this.movementSpeed.getDouble(Constants.SwerveConstants.defaultAngularSpeed);
    }
    public boolean getIsFieldRelative() {
        return this.movementSpeed.getBoolean(Constants.SwerveConstants.defaultFieldOriented);
    }

    //入力受付ウィジェット用のセッター
    public void setMovementSpeed(double speed) {
        this.movementSpeed.setDouble(speed);
    }
    public void setRotationSpeed(double rotationSpeed) {
        this.rotationSpeed.setDouble(rotationSpeed);
    }
    public void setIsFieldRelative(boolean isFieldRelative) {
        this.isFieldRelative.setBoolean(isFieldRelative);
    }
}
