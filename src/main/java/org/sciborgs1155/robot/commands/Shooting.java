package org.sciborgs1155.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static java.lang.Math.atan;
import static java.lang.Math.pow;
import static org.sciborgs1155.robot.Constants.Field.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.pivot.PivotConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.shooter.ShooterConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.shooter.ShooterConstants.RADIUS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.DriveConstants;
import org.sciborgs1155.robot.feeder.Feeder;
import org.sciborgs1155.robot.pivot.Pivot;
import org.sciborgs1155.robot.pivot.PivotConstants;
import org.sciborgs1155.robot.shooter.Shooter;

public class Shooting implements Logged{

   /**
   * *The conversion between shooter tangential velocity and note launch velocity. Perhaps. This may
   * also account for other errors with our model.*
   * 
   * "siggy's constant" copied from the original robot code.
   */
  public static final DoubleEntry siggysConstant = Tuning.entry("/Robot/Siggy's Constant", 4.42);
  public static final Distance MAX_DISTANCE = Meters.of(5);
  private static final InterpolatingDoubleTreeMap shotVelLookup = new InterpolatingDoubleTreeMap();

  private final Shooter shooter;
  private final Feeder feeder;
  private final Pivot pivot;
  private final Drive drive;

  public Shooting(Shooter shooter, Pivot pivot, Feeder feeder, Drive drive) {

    this.shooter = shooter;
    this.feeder = feeder;
    this.pivot = pivot;
    this.drive = drive;

    shotVelLookup.put(0.0, 300.0);
    shotVelLookup.put(1.0, 450.0);
    shotVelLookup.put(4.0, MAX_VELOCITY.in(RadiansPerSecond));

  }

  /**
   * ok bunch of notes to interpret
   */

/**
   * winds the shooter BEFORE the note is fed
   * @param desiredVel
   * @return a command shooting the note at the desiredVel
   */
  public Command shoot(AngularVelocity desiredVel) {
    return shoot(() -> desiredVel.in(RadiansPerSecond), () -> true);
  }


  /**
   * same as above but only runs feeder when shootCondition is met 
   * @param desiredVelocity
   * @param shootCondition
   * @return 
   */
  public Command shoot(DoubleSupplier desiredVelocity, BooleanSupplier shootCondition) {
    return Commands.waitUntil(
        () -> shooter.atVelocity(desiredVelocity.getAsDouble()) && shootCondition.getAsBoolean())
        .andThen(feeder.eject())
        .deadlineFor(shooter.runShooter(desiredVelocity));
    }

  /**
   * pivots to a target angle, winds shooter then feeds note.
   * @param targetAngle
   * @param targetAngularVel
   * @return
   */
  public Command PivotShoot(DoubleSupplier targetAngle, DoubleSupplier targetAngularVel) {
    return shoot(targetAngularVel, () -> pivot.atPos(targetAngle.getAsDouble()))
    .deadlineFor(pivot.runPivot(targetAngle));

  }

  /**
   * uses the above pivotshoot method to run the command using this methods parameters
   * @param targetAngle
   * @param targetAngularVelocity
   * @return
   */
  public Command PivotShoot(Angle targetAngle, AngularVelocity targetAngularVelocity) {
    return PivotShoot(() -> targetAngle.in(Radians), () -> targetAngularVelocity.in(RadiansPerSecond));
  }

  /**
   * "shoots stationary with the correct pivot angle and vel"
   * @return
   */
  public Command PivotShoot() {
    return PivotShoot(
      () -> pitchFromNoteVel(calcNoteVel()), 
      () -> rotationalVelFromNoteVel(calcNoteVel()));
  }

  public Command Aim() {
    return pivot.runPivot(() -> pitchFromNoteVel(calcNoteVel()));
  }

    /**
   * Shoots while driving at a manually inputted translational velocity.
   *
   * @param vx The field relative x velocity to drive in.
   * @param vy The field relative y velocity to drive in.
   * @return A command to shote while moving.
   */
  public Command shootWhileDriving(InputStream vx, InputStream vy) {
    return shoot(
            () -> rotationalVelFromNoteVel(calcNoteVel()),
            () ->
                pivot.atPos(pitchFromNoteVel(calcNoteVel()))
                    && atYaw(yawFromNoteVel(calcNoteVel())))
        .deadlineFor(
            drive.drive(
                vx.scale(0.5),
                vy.scale(0.5),
                () -> yawFromNoteVel(calcNoteVel(Seconds.of(0.2)))),
            pivot.runPivot(() -> pitchFromNoteVel(calcNoteVel())));
  }


  public static Pose2d robotPoseFacingSpeaker(Translation2d robotTranslation) {
    return new Pose2d(
        robotTranslation,
        translationToSpeaker(robotTranslation)
            .getAngle()
            .plus(Rotation2d.fromRadians(Math.PI / 2)));
  }
  
  public Vector<N3> calcNoteVel() {
    return calcNoteVel(drive.pose());
  }

  public Vector<N3> calcNoteVel(Time predictionTime) {
    return calcNoteVel(
      predictedPose(drive.pose(), drive.getFieldRelChassisSpeeds(), predictionTime)
    );
  }

   /**
   * Calculates a vector for the desired note velocity relative to the robot for it to travel into
   * the speaker, accounting for the robot's current motion.
   *
   * @return A 3d vector representing the desired note initial velocity.
   */
  public Vector<N3> calcNoteVel(Pose2d robotPose) {
    ChassisSpeeds speeds = drive.getFieldRelChassisSpeeds();
    Vector<N3> robotVelocity =
        VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0);
    Translation2d difference = translationToSpeaker(robotPose.getTranslation());
    double shotVelocity = calculateStationaryVelocity(difference.getNorm());
    Rotation3d noteOrientation =
        new Rotation3d(
            0,
            -calculateStationaryPitch(
                robotPoseFacingSpeaker(robotPose.getTranslation()), shotVelocity, pivot.pos()),
            difference.getAngle().getRadians());
    // rotate unit forward vector by note orientation and scale by our shot velocity
    Vector<N3> noteVelocity =
        new Translation3d(1, 0, 0).rotateBy(noteOrientation).toVector().unit().times(shotVelocity);

    return noteVelocity.minus(robotVelocity);
  }

  /**
   * returns a pose2d pose prediction using the current robot pose and speed after a period of time.
   * @param robotPose
   * @param speeds
   * @param predictionTime
   * @return
   */
  public static Pose2d predictedPose(Pose2d robotPose, ChassisSpeeds speeds, Time predictionTime) {
    Vector<N3> current =
    VecBuilder.fill(robotPose.getX(), robotPose.getY(), robotPose.getRotation().getRadians());
    Vector<N3> velocity =
    VecBuilder.fill(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    Vector<N3> predicted = current.plus(velocity.times(predictionTime.in(Seconds)));
    return new Pose2d(predicted.get(0), predicted.get(1), Rotation2d.fromRadians(predicted.get(2)));  
  }

  /**
   * returns the current pose of the shooter
   * @return
   */
  @Log.NT
  public Pose3d shooterPose() {
    return new Pose3d(drive.pose())
    .transformBy(pivot.transform())
    .transformBy(PivotConstants.SHOOTER_FROM_AXLE);
  }

  public static Pose3d shooterPose(Transform3d pivot, Pose2d robot) {
    return new Pose3d(robot)
    .transformBy(pivot)
    .transformBy(PivotConstants.SHOOTER_FROM_AXLE);

  }

  /**
   * checks if the robot can currently make a shot 
   * @return
   */
  @Log.NT
  public boolean inRange() {
    Vector<N3> shot = calcNoteVel();
    double pitch = pitchFromNoteVel(shot);
    return MIN_ANGLE.in(Radians) < pitch
      && pitch < MAX_ANGLE.in(Radians)
      && Math.abs(rotationalVelFromNoteVel(shot)) < MAX_VELOCITY.in(RadiansPerSecond)
      && translationToSpeaker(drive.pose().getTranslation()).getNorm() < MAX_DISTANCE.in(Meters);

  }

  public boolean atYaw(Rotation2d yaw) {
    double tolerance = DriveConstants.Rotation.TOLERANCE.in(Radians) * (1 - yaw.getSin());
    Rotation2d diff = drive.heading().minus(yaw);
    return Math.abs(atan(diff.getTan())) < tolerance;
    
  }


  /**
   * returns a pivot angle using the note's initial velocity vector & the robots relative inital velocity
   * @param velocity
   * @return
   */
  public static double pitchFromNoteVel(Vector<N3> velocity) {
    return Math.atan(velocity.get(2) / VecBuilder.fill(velocity.get(0), velocity.get(1)).norm());
  }


  /**
   * returns the "target robot heading" using the note's initial velocity vector and the robots initial relative velocity
   * @param velocity
   * @return
   */
  public static Rotation2d yawFromNoteVel(Vector<N3> velocity) {
    return Rotation2d.fromRadians(Math.PI).plus(new Rotation2d(velocity.get(0), velocity.get(1)));

  }


  /**
   * "Calculates magnitude of initial velocity vector of note, in radians per second. If given a
   * robot relative initial velocity vector, the return value will be the target flywheel speed
   * (ish)."
   *
   * @param velocity Note initial velocity vector relative to the robot
   * @return Flywheel speed (rads / s)
   */
  public static double rotationalVelFromNoteVel(Vector<N3> velocity)  {
    return velocity.norm() / RADIUS.in(Meters) * siggysConstant.get();
  }

  /**
   * converts the flywheel speed to its relative note speed
   * @param flywheelSpeed
   * @return
   */
  public static double flywheelToNoteSpeed(double flywheelSpeed) {
    return flywheelSpeed * RADIUS.in(Meters) / siggysConstant.get();
  }

  public static Translation2d translationToSpeaker(Translation2d robotTranslation) {
    return speaker().toTranslation2d().minus(robotTranslation);
  }

  public static double calculateStationaryVelocity(double distance) {
    return flywheelToNoteSpeed(shotVelLookup.get(distance));
  }

    /**
   * Calculates a stationary pitch from a pose so that the note goes into the speaker.
   *
   * @param shooterPose The pose of the shooter.
   * @param velocity The magnitude of velocity to launch the note at.
   * @return The pitch to shoot the note at.
   */
  public static double calculateStationaryPitch(
      Pose2d robotPose, double velocity, double prevPitch) {
    return calculateStationaryPitch(robotPose, velocity, prevPitch, 0);
  }

  private static double calculateStationaryPitch(
      Pose2d robotPose, double velocity, double prevPitch, int i) {
    double G = 9.81;
    Translation3d shooterTranslation =
        shooterPose(Pivot.transform(-prevPitch), robotPose).getTranslation();
    double dist = translationToSpeaker(shooterTranslation.toTranslation2d()).getNorm();
    double h = speaker().getZ() - shooterTranslation.getZ();
    double denom = (G * pow(dist, 2));
    double rad =
        pow(dist, 2) * pow(velocity, 4)
            - G * pow(dist, 2) * (G * pow(dist, 2) + 2 * h * pow(velocity, 2));
    double pitch = Math.atan((1 / (denom)) * (dist * pow(velocity, 2) - Math.sqrt(rad)));
    if (Math.abs(pitch - prevPitch) < 0.005 || i > 50) {
      return pitch;
    }
    return calculateStationaryPitch(robotPose, velocity, pitch, i + 1);
  }

}






