package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.teleop;
import static org.sciborgs1155.lib.Assertion.EqualityAssertion;
import static org.sciborgs1155.lib.Assertion.eAssert;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Set;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;


public class Pivot extends SubsystemBase implements Logged, AutoCloseable{

    private final PivotIO hardware;
    private final SysIdRoutine sysId;


    private final ProfiledPIDController pid = 
    new ProfiledPIDController(
        kP, 
        kI, 
        kD, 
        new TrapezoidProfile.Constraints(MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCEL.in(RadiansPerSecondPerSecond)));

    private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

    /*
     * visualization
     */
    @Log.NT
    private final PivotVisualizer positionVisualizer = new PivotVisualizer(new Color8Bit(255, 0, 0));

    @Log.NT
    private final PivotVisualizer setpointVisualizer = new PivotVisualizer(new Color8Bit(0, 0, 255));

    /**
     * creates a new real/sim pivot depending on whether robot connection is present
     * @return RealPivot or SimPivot
     */
    public static Pivot create() {
        return Robot.isReal() ? 
        new Pivot(new RealPivot()) 
        : new Pivot(new SimPivot());
    }

    /**
     * creates a new nonexistant pivot
     * @return NoPivot
     */
    public static Pivot none() {
        return new Pivot(new NoPivot());
      }

    public Pivot(PivotIO pivot) {
        this.hardware = pivot;
        sysId =
            new SysIdRoutine(new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(3), Seconds.of(6)),
            new SysIdRoutine.Mechanism(v -> pivot.setVoltage(v.in(Volts)), null, null));

        pid.reset(MAX_ANGLE.in(Radians));
        pid.setTolerance(POSITION_TOLERANCE.in(Radians));

        SmartDashboard.putData("pivot quasistatic forward", quasistaticForward());
        SmartDashboard.putData("pivot quasistatic backward", quasistaticBack());
        SmartDashboard.putData("pivot dynamic forward", dynamicForward());
        SmartDashboard.putData("pivot dynamic backward", dynamicBack());

        setDefaultCommand(
            run(() -> update(MAX_ANGLE.in(Radians)))
                .until(() -> pid.getGoal().position > MAX_ANGLE.in(Radians))
                .andThen(run(() -> pivot.setVoltage(0)))
                .withName("default position"));
    
        teleop().or(autonomous()).onTrue(Commands.runOnce(() -> pid.reset(hardware.getPosition())));

    }

    //commands
    public Command runPivot(DoubleSupplier goalAngle) {
        return run(() -> update(goalAngle.getAsDouble())).withName("go to").asProxy();
    }

    public Command runPivot(Angle goal) {
        return runPivot(() -> goal.in(Radians));
    }
        

    public Command climb() {
        return run(() -> hardware.setVoltage(12))
        .beforeStarting(() -> hardware.setCurrentLimit(CLIMBER_CURRENT_LIMIT))
        .finallyDo(() -> hardware.setCurrentLimit(CURRENT_LIMIT))
        .withName("climb");
    }

    public Command manual(DoubleSupplier stickInput) {
        return runPivot(
            InputStream.of(stickInput)
                .scale(MAX_VELOCITY.in(RadiansPerSecond) / 4)
                .scale(Constants.PERIOD.in(Seconds))
                .add(() -> pid.getGoal().position)
        );
    }

    public Command setGoal(DoubleSupplier goalPos) {
        return runOnce(() -> pid.setGoal(goalPos.getAsDouble()));
    }

    public Rotation3d rotation() {
        return new Rotation3d(0, hardware.getPosition(), 0);
    }

    public Rotation3d goal() {
        return new Rotation3d(0, pid.getGoal().position, 0);
    }

    public Rotation3d setpoint() {
        return new Rotation3d(0, pid.getSetpoint().position, 0);
    }

    public Transform3d transform() {
        return new Transform3d(AXLE_FROM_CHASSIS, rotation());
    }

    public static Transform3d transform(double position) {
        return new Transform3d(AXLE_FROM_CHASSIS, new Rotation3d(0, position, 0));
    }

    public double pos() {
        return hardware.getPosition();
    }

    public boolean atGoal() {
        return pid.atGoal();
    }

    public boolean atPos(double pos) {
        return Math.abs(pos - hardware.getPosition()) < POSITION_TOLERANCE.in(Radians);
    }


    public Command quasistaticForward() {
        return sysId
        .quasistatic(Direction.kForward)
        .until(() -> hardware.getPosition() > POSITION_TOLERANCE.in(Radians) - 0.2);
    }
    
    public Command quasistaticBack() {
        return sysId
        .quasistatic(Direction.kReverse)
        .until(() -> hardware.getPosition() > POSITION_TOLERANCE.in(Radians) + 0.2);

    }

    public Command dynamicForward() {
        return sysId
        .dynamic(Direction.kForward)
        .until(() -> hardware.getPosition() > POSITION_TOLERANCE.in(Radians) - 0.2);

    }

    public Command dynamicBack() {
        return sysId
        .dynamic(Direction.kForward)
        .until(() -> hardware.getPosition() > POSITION_TOLERANCE.in(Radians) + 0.2);

    }

    /**
     * angles the pivot to a position using profiledPIDcontroller
     * @param goalAngle
     */
    private void update(double goalAngle) {
        double goal = 
            Double.isNaN(goalAngle)
                ? MAX_ANGLE.in(Radians)
                : MathUtil.clamp(goalAngle, MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
        var prevSetPoint = pid.getSetpoint();
        double feedback = pid.calculate(hardware.getPosition(), goal);
        double accel = (pid.getSetpoint().position - prevSetPoint.velocity) / PERIOD.in(Seconds);
        double feedforward = ff.calculate(pid.getSetpoint().position + Math.PI, pid.getSetpoint().velocity, accel);
        hardware.setVoltage(feedback + feedforward);

    }

    @Override
    public void periodic() {
        positionVisualizer.setState(hardware.getPosition());
        setpointVisualizer.setState(setpoint().getY());
    }

    @Override
    public void close() throws Exception {
      positionVisualizer.close();
      setpointVisualizer.close();
      hardware.close();
    }

    public Test test(Angle goal) {
        Command testCom = runPivot(goal).until(() -> atPos(goal.in(Radians))).withTimeout(2);
        EqualityAssertion atGoal = 
        eAssert("pivot test angle (degrees)", () -> goal.in(Degrees), () -> Units.radiansToDegrees(pos()),POSITION_TOLERANCE.in(Degrees));
        return new Test(testCom, Set.of(atGoal));

    }

}
    