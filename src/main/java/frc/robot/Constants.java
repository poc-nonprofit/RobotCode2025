// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    //コントローラーのポート番号とスティックのデッドゾーン
    public static class OperatorConstants
    {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double CONTROLLER_LEFT_DEADZONE_X = 0.05;
        public static final double CONTROLLER_LEFT_DEADZONE_Y = 0.05;
        public static final double CONTROLLER_RIGHT_DEADZONE_X = 0.05;
    }

    //各モータードライバ/エンコーダーのCAN ID
    public static class DriveMotorPort{
        public static final int FRONT_LEFT = 1;
        public static final int FRONT_RIGHT = 3;
        public static final int REAR_LEFT = 7;
        public static final int REAR_RIGHT = 5;
    }
    public static class SteeringMotorPort{
        public static final int FRONT_LEFT = 2;
        public static final int FRONT_RIGHT = 4;
        public static final int REAR_LEFT = 8;
        public static final int REAR_RIGHT = 6;
    }
    public static class CANCoderPort{
        public static final int FRONT_LEFT = 11;
        public static final int FRONT_RIGHT = 12;
        public static final int REAR_LEFT = 14;
        public static final int REAR_RIGHT = 13;
    }
    public static class ElevatorMotor {
        public static final int ELEVATOR_MOTOR_PORT = 20;
        public static final int ELEVATOR_SERVO_PORT = 0;
        public static final double DEFAULT_ELEVATOR_SPEED = 0.20;
    }
    //サーボの設定 (角度)
    public static class ElevatorServoConf {
        public static final double DEFAULT_ANGLE = 0.0; //初期角度(度)
        public static final double ROTATION_ANGLE = 120.0; //動いたときの角度
        public static final double WAITING_TIME = 3.0; //
    }


    //各モジュールの位置計算用(あんまり使ってない
    public enum moduleLocations {
        FL(0.5, 0.5),
        FR(0.5, -0.5),
        RL(-0.5, 0.5),
        RR(-0.5, -0.5);

        private final Translation2d value;

        moduleLocations(double x, double y) {
            this.value = new Translation2d(x, y);
        }

        public Translation2d getValue() {
            return value;
        }
    }

    public static class SwerveConstants {
        // ホイール半径5.08 cm, 2 in
        public static final double wheelRaduis = Units.Meters.convertFrom(5.08, Units.Centimeter);
        //モーター内蔵のエンコーダーの解像度(42カウント/一周)
        public static final int brushlessMotorResolution = 42;
        //速度のデフォルト値(%)
        public static final double defaultSpeed = 0.2;
        //最大速度 (%)
        public static final double maxMovementSpeed = 1.0;
        //角速度のデフォルト値,30 deg/s
        public static double defaultAngularSpeed = Math.PI / 6;
        //最大角速度, 360deg/s
        public static final double maxAngularSpeed = 2*Math.PI;
        //フィールド相対のデフォルト
        public static final boolean defaultFieldOriented = false;
        //モーターギア : ホイールギア比 = 8.14:1
        public static double gearRatio = 8.14;
        public static double defaultPeriod = TimedRobot.kDefaultPeriod;

        //スワーブ車輪の角度のオフセット
        public static class ModuleOffset{
            public static final Rotation2d FL = Rotation2d.kZero;
            public static final Rotation2d FR = Rotation2d.kZero;
            public static final Rotation2d RL = Rotation2d.kZero;
            public static final Rotation2d RR = Rotation2d.kZero;
        }

    }
}

