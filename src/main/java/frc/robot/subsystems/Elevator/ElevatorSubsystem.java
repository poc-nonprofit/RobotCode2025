package frc.robot.subsystems.Elevator;


import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
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



    public ElevatorSubsystem() {
        this.elevatorMotor = new SparkMax(Constants.ElevatorMotor.ELEVATOR_MOTOR_PORT, SparkLowLevel.MotorType.kBrushless);
        this.elevatorServo = new Servo(Constants.ElevatorMotor.ELEVATOR_SERVO_PORT);

    }

    public void moveElevator(double input) {
        this.elevatorMotor.set(input * Constants.ElevatorMotor.ELEVATOR_SPEED);
    }
    public Command releaseStopperServo() {
        return runOnce(()->{
            this.elevatorServo.set(0.5);
            Timer.delay(3.0);
            this.elevatorServo.set(0);
        });
    }

    public void setControllerBindings(CommandXboxController controller){
        controller.a().onTrue(this.releaseStopperServo());
    }
}

