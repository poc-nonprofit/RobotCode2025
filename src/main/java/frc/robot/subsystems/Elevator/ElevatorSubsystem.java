package frc.robot.subsystems.Elevator;


import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private final static ElevatorSubsystem INSTANCE = new ElevatorSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static ElevatorSubsystem getInstance() {
        return INSTANCE;
    }

    private final SparkMax elevatorMotor;
    private final Servo elevatorServo;

    //モーター速度
    private double elevatorSpeed = Constants.ElevatorMotor.DEFAULT_ELEVATOR_SPEED;

    //初期化
    public ElevatorSubsystem() {
        this.elevatorMotor = new SparkMax(Constants.ElevatorMotor.ELEVATOR_MOTOR_PORT, SparkLowLevel.MotorType.kBrushless);
        this.elevatorServo = new Servo(Constants.ElevatorMotor.ELEVATOR_SERVO_PORT);

    }

    public void moveElevator(double input) {
        this.elevatorMotor.set(input * Constants.ElevatorMotor.DEFAULT_ELEVATOR_SPEED);
    }
    //留め具のサーボを動かす
    public Command releaseStopperServo() {
        return runOnce(()->{
            this.elevatorServo.set( Constants.ElevatorServoConf.DEFAULT_ANGLE / 270.0);
            Timer.delay(Constants.ElevatorServoConf.WAITING_TIME);
            this.elevatorServo.set(Constants.ElevatorServoConf.ROTATION_ANGLE / 270.0);
        });
    }
    //エレベーターの速度を変える
    public Command changeSpeed(boolean increase) {
        return runOnce(()->{
            double speed = this.elevatorSpeed + (increase ? 0.05 : -0.05);
            this.elevatorSpeed = MathUtil.clamp(speed,0,1);
        });
    }

    public void setControllerBindings(CommandXboxController controller){
        controller.a().onTrue(this.releaseStopperServo());
        controller.povUp().onTrue(this.changeSpeed(true));
        controller.povDown().onTrue(this.changeSpeed(false));
    }
}

