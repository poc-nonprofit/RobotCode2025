// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveTrain.SwerveDriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


//Autonomous 用のコマンドクラス
public final class Autos
{
    //コマンドを作成して実行
    public static Command runOnlyForwardAuto(SwerveDriveTrainSubsystem driveTrain)
    {
        return new RunCommand(()->{
            //ちょっとづつ前に進むだけ
            driveTrain.drive(
                -0.5,0,0
            );
        },driveTrain);
        //return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
    }
}
