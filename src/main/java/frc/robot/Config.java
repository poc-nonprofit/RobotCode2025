package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class Config {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig steeringConfig = new SparkMaxConfig();

    static {
        double drivingFactor = Constants.SwerveConstants.wheelRaduis * 2 * Math.PI / Constants.SwerveConstants.gearRatio; //挙動に応じて適宜調整
        double steeringFactor = 2 * Math.PI;
        double drivingVelocityFeedForward = 1;

        drivingConfig
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(50);

        drivingConfig.encoder.
            positionConversionFactor(drivingFactor) // meters
            .velocityConversionFactor(drivingFactor / 60.0); // meters per second

        drivingConfig.closedLoop.
            feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pid(1.0, 0, 0) //ここは係数をいい感じに調整
            .outputRange(-1, 1);

        steeringConfig
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(50);
        drivingConfig.encoder
            .positionConversionFactor(steeringFactor)
            .velocityConversionFactor(steeringFactor / 60.0);

        steeringConfig.closedLoop.
            feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pid(1, 0, 0)
            .outputRange(-1, 1)
            //例えば,350度から10度に向かう場合に,0度経由の遷移を可能にする
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, steeringFactor);

    }
}
