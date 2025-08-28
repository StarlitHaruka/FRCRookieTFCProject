package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.sciborgs1155.lib.FaultLogger.check;
import static org.sciborgs1155.lib.FaultLogger.register;
import static org.sciborgs1155.robot.shooter.ShooterConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.robot.Ports;

public class RealShooter implements ShooterIO {
  private final SparkFlex motor;
  private final RelativeEncoder shooterEncoder;
  private final SparkFlexConfig config;

  public RealShooter(int id, boolean inverted) {
    motor = new SparkFlex(Ports.Shooter.MOTOR, MotorType.kBrushless);
    shooterEncoder = motor.getEncoder();
    config = new SparkFlexConfig();

    config.apply(config.idleMode(IdleMode.kCoast).smartCurrentLimit((int) ShooterConstants.CURRENT_LIMIT.in(Amps)));
    config.apply(config.absoluteEncoder.inverted(inverted));
    config.apply(
        config
            .encoder
            .positionConversionFactor(POSITION_FACTOR.in(Radians))
            .velocityConversionFactor(VELOCITY_FACTOR.in(RotationsPerSecond)));

    config.apply(
        SparkUtils.getSignalsConfigurationFrameStrategy(
            Set.of(
                SparkUtils.Data.POSITION, SparkUtils.Data.VELOCITY, SparkUtils.Data.APPLIED_OUTPUT),
            Set.of(Sensor.INTEGRATED),
            false));

    check(
        motor,
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));

    check(
        motor,
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));

    register(motor);
  }

  @Override
  public void setVoltage(double voltage) {
    // sets volt
    motor.setVoltage(voltage);
  }

  @Override
  public double velocity() {
    // gets vel
    return shooterEncoder.getVelocity();
  }

  @Override
  public void close() {
    motor.close();
  }
}
