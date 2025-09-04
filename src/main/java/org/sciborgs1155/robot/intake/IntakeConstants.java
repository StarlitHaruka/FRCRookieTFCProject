package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public final class IntakeConstants {
  public static final double INTAKE_SPEED = 0.8;
  public static final Current CURRENT_LIMIT = Amps.of(50);
  public static final Time RAMP_TIME = Milliseconds.of(50);
  public static final Time DEBOUNCE_TIME = Seconds.of(0);
  public static final Time INTAKE_FAST_PERIOD = Milliseconds.of(5);

  public static final Current STALL_THRESHOLD = Amps.of(40);
}
