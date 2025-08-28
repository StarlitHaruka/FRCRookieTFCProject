package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SimShooter implements ShooterIO {
  private final FlywheelSim wheel;

  public SimShooter(double kV, double kA) {

    wheel = 
      new FlywheelSim(null, null, null);

  }

  // empty for now x3

  @Override
  public void setVoltage(double voltage) {
    wheel.setInputVoltage(voltage);
    wheel.update(PERIOD.in(Seconds));
  }

  @Override
  public double velocity() {
    return wheel.getAngularVelocityRadPerSec();
  }

  @Override
  public void close() {}
}
