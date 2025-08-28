package org.sciborgs1155.robot.shooter;

public interface ShooterIO {

  void setVoltage(double voltage);

  double velocity();

  void close();
  
}
