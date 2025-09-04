package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.shooter.ShooterConstants.FF;
import org.sciborgs1155.robot.shooter.ShooterConstants.PID;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.commands.Shooting;

import static org.sciborgs1155.robot.shooter.ShooterConstants.DEFAULT_VELOCITY;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.shooter.ShooterConstants.VELOCITY_TOLERANCE;

import java.util.function.DoubleSupplier;



public class Shooter extends SubsystemBase implements Logged, AutoCloseable {
  private final ShooterIO hardware;

  @Log.NT private double setpoint;

  private final SimpleMotorFeedforward hardwareFeedforward =
      new SimpleMotorFeedforward(FF.kS, FF.kV, FF.kA);

  private final PIDController hardwarePIDController = new PIDController(PID.kP, PID.kI, PID.kD);

  public static Shooter create() {
    if (Robot.isReal()) {
      return new Shooter(null);
    } else {
      return new Shooter(null);
    }

  }

  public Shooter(ShooterIO hardware) {
    this.hardware = hardware;

    hardwarePIDController.setTolerance(VELOCITY_TOLERANCE.in(RadiansPerSecond));

    setDefaultCommand(run(() -> update(0)));
  }

  public Command runShooter(double velocity) {
    return runShooter(() -> velocity);
  }

    @Log.NT
  public double tangentialVelocity() {
    return Shooting.flywheelToNoteSpeed(rotationalVelocity());
  }

  public double rotationalVelocity() {
    return (topVelocity()) / 2.0;
  }


  public void setVoltage(double voltage) {
    hardware.setVoltage(voltage);
  }

  public double velocity() {
    return hardware.velocity();
  }

  public boolean atVelocity(double velocity) {
    return Math.abs(velocity - topVelocity()) < VELOCITY_TOLERANCE.in(RadiansPerSecond);
  }

    @Log.NT
  public double topVelocity() {
    return hardware.velocity();
  }

  public Command runShooter(DoubleSupplier vel) {
    return run(() -> update(vel.getAsDouble())).withName("running shooter");
  }

  public void update(double velocitySetpoint) {
    double velocity =
        Double.isNaN(velocitySetpoint)
            ? DEFAULT_VELOCITY.in(RadiansPerSecond)
            : MathUtil.clamp(
                velocitySetpoint,
                -MAX_VELOCITY.in(RadiansPerSecond),
                MAX_VELOCITY.in(RadiansPerSecond));
    double FF = hardwareFeedforward.calculate(velocity);
    double FB = hardwarePIDController.calculate(velocity, velocity);
    hardware.setVoltage(MathUtil.clamp(FF + FB, -12, 12));
    setpoint = velocity;
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }

  


  
}
