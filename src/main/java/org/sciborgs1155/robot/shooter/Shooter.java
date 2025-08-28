package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import org.sciborgs1155.robot.shooter.ShooterConstants.FF;
import org.sciborgs1155.robot.shooter.ShooterConstants.PID;

import static org.sciborgs1155.robot.shooter.ShooterConstants.DEFAULT_VELOCITY;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.shooter.ShooterConstants.VELOCITY_TOLERANCE;


public class Shooter extends SubsystemBase implements Logged {
  private final ShooterIO hardware;

  private double setpoint;

  private final SimpleMotorFeedforward hardwareFeedforward =
      new SimpleMotorFeedforward(FF.kS, FF.kV, FF.kA);

  private final PIDController hardwarePIDController = new PIDController(PID.kP, PID.kI, PID.kD);

  public Shooter(ShooterIO hardware) {
    this.hardware = hardware;

    hardwarePIDController.setTolerance(VELOCITY_TOLERANCE.in(RadiansPerSecond));

    setDefaultCommand(run(() -> update(0)));
  }

  public void setVoltage(double voltage) {
    hardware.setVoltage(voltage);
  }

  public double velocity() {
    return hardware.velocity();
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


  
}
