package org.sciborgs1155.robot.shooter;

public class NoShooter implements ShooterIO {

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double velocity() {
    return 0;
  }

  @Override
  public void close() {
    
  }
}
