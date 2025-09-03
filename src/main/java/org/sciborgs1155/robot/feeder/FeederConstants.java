package org.sciborgs1155.robot.feeder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class FeederConstants {
  public static final Current CURRENT_LIMIT = Amps.of(45);
  public static final LinearVelocity FEEDER_VELOCITY = MetersPerSecond.of(0.8);
  public static final double GEARING = 18.0 / 64.0;
  public static final Distance RADIUS = Inches.of(0.63);

  public static final double POWER = 0.5;
  public static final Time TIMEOUT = Seconds.of(0.5);
  public static final Time DEBOUNCE_TIME = Seconds.of(0);
  public static final Time RAMP_TIME = Milliseconds.of(50);

  public static final double kS = 0;
  public static final double kV = 0.2;
  public static final double kA = 0.7;
}
