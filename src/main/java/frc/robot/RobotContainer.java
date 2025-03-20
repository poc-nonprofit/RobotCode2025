// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveTrain.SwerveDriveTrainSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    //スワーブを管理するサブシステムのインスタンス
    private final SwerveDriveTrainSubsystem swerveDriveTrainSubsystem = SwerveDriveTrainSubsystem.getInstance();
    //カメラサーバー
    private final CameraSubsystem cameraSubsystem = CameraSubsystem.getInstance();
    private final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
        new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController =
        new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        //カメラ始動
        //cameraSubsystem.start();

        //デフォルトコマンド. ずっと実行
        /*swerveDriveTrainSubsystem.setDefaultCommand(
            new RunCommand(() -> swerveDriveTrainSubsystem.drive(
                driverController.getLeftY(), //左スティック縦軸
                driverController.getLeftX(), //左スティック横軸
                driverController.getRightX() //右スティック横軸
            ),swerveDriveTrainSubsystem)
        );*/
        //ドライバの要望により,スティック逆
        swerveDriveTrainSubsystem.setDefaultCommand(
            new RunCommand(() -> swerveDriveTrainSubsystem.drive(
                driverController.getRightY(), //右スティック縦軸
                driverController.getRightX(), //右スティック横軸
                driverController.getLeftX() //左スティック横軸
            ),swerveDriveTrainSubsystem)
        );
        //エレベーター用
        elevatorSubsystem.setDefaultCommand(
            new RunCommand(()-> elevatorSubsystem.moveElevator(
                -operatorController.getLeftY()
            ),elevatorSubsystem)
        );
    }


    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        //速度調整用のボタンのコマンドをバインド(十字キー)
        swerveDriveTrainSubsystem.setControllerBindings(driverController);
        elevatorSubsystem.setControllerBindings(operatorController);
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //Auto用のコマンド
        return Autos.runOnlyForwardAuto(swerveDriveTrainSubsystem);
    }
}
