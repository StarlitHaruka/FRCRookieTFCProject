package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class PivotConstants {
  public static final double MOTOR_GEARING = 12.0 / 64.0 * 20.0 / 70.0 * 36.0 / 56.0 * 16.0 / 54.0;
  public static final double THROUGHBORE_GEARING = 16.0 / 54.0;

  public static final Angle POSITION_FACTOR = Rotations.of(THROUGHBORE_GEARING);
  public static final AngularVelocity VELOCITY_FACTOR = POSITION_FACTOR.per(Minute);

  /** Offset from the center of the robot to the pivot's axis of rotation */
  public static final Translation3d AXLE_FROM_CHASSIS =
      new Translation3d(Inches.of(-10.465), Inches.zero(), Inches.of(25));

  /** Offset from the pivot's axis of rotation to the shooter beambreak. */
  public static final Transform3d SHOOTER_FROM_AXLE =
      new Transform3d(
          new Translation3d(Inches.of(9.118), Inches.zero(), Inches.of(5.868)), new Rotation3d());

  public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.17845);

  public static final Angle POSITION_TOLERANCE = Degrees.of(0.8);

  public static final Mass MASS = Kilograms.of(1);
  public static final Distance LENGTH = Inches.of(16);

  public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(9.0);
  public static final AngularAcceleration MAX_ACCEL = RadiansPerSecond.per(Second).of(13.0);

  public static final Angle STARTING_ANGLE = Degrees.of(63.3);

  public static final Angle MIN_ANGLE = Degrees.of(-45.7);
  public static final Angle MAX_ANGLE = STARTING_ANGLE.minus(Degrees.of(4.1)); // 1.2

  public static final Angle PRESET_SUBWOOFER_ANGLE = STARTING_ANGLE;
  public static final Angle AMP_ANGLE = Radians.of(-0.645);
  public static final Angle PRESET_PODIUM_ANGLE = Radians.of(0.5);

  public static final Angle PRESET_CLIMBING_ANGLE = Radians.of(-0.195);

  public static final Current CURRENT_LIMIT = Amps.of(30);
  public static final Current CLIMBER_CURRENT_LIMIT = Amps.of(60);

  public static final Angle UNDERFEED_ANGLE = Radians.of(-0.027);
  public static final Angle OVERFEED_ANGLE = Radians.of(0.619);
  public static final double kP = 8.0;
  public static final double kI = 0.0;
  public static final double kD = 0.5;

  public static final double kS = 0.14296;
  public static final double kV = 1.7305;
  public static final double kA = 0.01;
  public static final double kG = 0.12055;
}
