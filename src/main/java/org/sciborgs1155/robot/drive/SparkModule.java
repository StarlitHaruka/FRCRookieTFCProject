package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.FaultLogger.*;
import static org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.COUPLING_RATIO;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Set;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class SparkModule implements ModuleIO {
  private final SparkFlex driveMotor; // NEO Vortex
  private final SparkFlexConfig driveMotorConfig;
  private final SparkMax turnMotor; // NEO 550
  private final SparkMaxConfig turnMotorConfig;

  private final RelativeEncoder driveEncoder;
  private final SparkAbsoluteEncoder turningEncoder;

  private final SparkClosedLoopController drivePID;
  private final SparkClosedLoopController turnPID;

  private final SimpleMotorFeedforward driveFF;

  private final Rotation2d angularOffset;

  private final String name;

  private double lastPosition;
  private double lastVelocity;

  @Log.NT private SwerveModuleState setpoint = new SwerveModuleState();

  public SparkModule(int drivePort, int turnPort, Rotation2d angularOffset, String name) {

    // Drive Motor

    driveMotor = new SparkFlex(drivePort, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    drivePID = driveMotor.getClosedLoopController();
    driveFF = new SimpleMotorFeedforward(Driving.FF.S, Driving.FF.V, Driving.FF.kA_linear);
    driveMotorConfig = new SparkFlexConfig();

    check(
        driveMotor,
        driveMotor.configure(
            driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));

    driveMotorConfig.apply(
        driveMotorConfig
            .closedLoop
            .pid(Driving.PID.P, Driving.PID.I, Driving.PID.D)
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder));

    driveMotorConfig.apply(
        driveMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) Driving.CURRENT_LIMIT.in(Amps)));

    driveMotorConfig.apply(
        driveMotorConfig
            .encoder
            .positionConversionFactor(Driving.POSITION_FACTOR.in(Meters))
            .velocityConversionFactor(Driving.VELOCITY_FACTOR.in(MetersPerSecond))
            .uvwAverageDepth(16)
            .uvwMeasurementPeriod(32));

    driveMotorConfig.apply(
        SparkUtils.getSignalsConfigurationFrameStrategy(
            Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
            Set.of(Sensor.INTEGRATED),
            false));

    check(
        driveMotor,
        driveMotor.configure(
            driveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));

    // Turn Motor

    turnMotor = new SparkMax(turnPort, MotorType.kBrushless);
    turningEncoder = turnMotor.getAbsoluteEncoder();
    turnPID = turnMotor.getClosedLoopController();
    turnMotorConfig = new SparkMaxConfig();

    check(
        turnMotor,
        turnMotor.configure(
            turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));

    turnMotorConfig.apply(
        turnMotorConfig
            .closedLoop
            .pid(Turning.PID.P, Turning.PID.I, Turning.PID.D)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-Math.PI, Math.PI)
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder));

    turnMotorConfig.apply(
        turnMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) Turning.CURRENT_LIMIT.in(Amps)));

    turnMotorConfig.apply(turnMotorConfig.absoluteEncoder.inverted(true));

    turnMotorConfig.apply(
        turnMotorConfig
            .absoluteEncoder
            .positionConversionFactor(Turning.POSITION_FACTOR.in(Radians))
            .velocityConversionFactor(Turning.VELOCITY_FACTOR.in(RadiansPerSecond))
            .averageDepth(2));

    turnMotorConfig.apply(
        SparkUtils.getSignalsConfigurationFrameStrategy(
            Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
            Set.of(Sensor.ABSOLUTE),
            false));

    check(
        turnMotor,
        turnMotor.configure(
            turnMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));

    register(driveMotor);
    register(turnMotor);

    resetEncoders();

    this.angularOffset = angularOffset;
    this.name = name;
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
    check(driveMotor);
    log("current", driveMotor.getOutputCurrent());
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turnMotor.setVoltage(voltage);
    check(turnMotor);
  }

  @Override
  public double drivePosition() {
    lastPosition = SparkUtils.wrapCall(driveMotor, driveEncoder.getPosition()).orElse(lastPosition);
    // account for rotation of turn motor on rotation of drive motor
    return lastPosition - turningEncoder.getPosition() * COUPLING_RATIO;
  }

  @Override
  public double driveVelocity() {
    lastVelocity = SparkUtils.wrapCall(driveMotor, driveEncoder.getVelocity()).orElse(lastVelocity);
    return lastVelocity;
  }

  @Override
  public Rotation2d rotation() {
    return Rotation2d.fromRadians(turningEncoder.getPosition()).minus(angularOffset);
  }

  @Override
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  @Override
  public void close() {
    driveMotor.close();
    turnMotor.close();
  }
}
