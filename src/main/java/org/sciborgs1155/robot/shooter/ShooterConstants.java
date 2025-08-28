package org.sciborgs1155.robot.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.units.measure.LinearVelocity;
// // import org.sciborgs1155.robot.commands.Shooting;

public class ShooterConstants {
  public static final double GEARING = 1;

  public static final Distance RADIUS = Inches.of(4);
  public static final Distance CIRCUMFERENCE = RADIUS.times(2 * Math.PI);

  public static final Current CURRENT_LIMIT = Amps.of(45);

  public static final Angle POSITION_FACTOR = Rotations.one();
  public static final AngularVelocity VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

  public static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(5);

  public static final AngularVelocity IDLE_VELOCITY = RadiansPerSecond.of(300);
  public static final AngularVelocity AMP_VELOCITY = RadiansPerSecond.of(150);

  public static final AngularVelocity DEFAULT_VELOCITY = RadiansPerSecond.of(550);
  // public static final LinearVelocity DEFAULT_NOTE_VELOCITY =
  //     MetersPerSecond.of(Shooting.flywheelToNoteSpeed(DEFAULT_VELOCITY.in(RadiansPerSecond)));
  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(630);
  // public static final LinearVelocity MAX_NOTE_VELOCITY =
  //     MetersPerSecond.of(Shooting.flywheelToNoteSpeed(MAX_VELOCITY.in(RadiansPerSecond)));

  public static final class PID {
    public static final double kP = 0.03;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }
   
  public static final class FF {
    public static final double kS = 0;
    public static final double kV = 0.016896;
    public static final double kA = 0.0031483;
  }

}