package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.autonomous;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.teleop;
import static org.sciborgs1155.lib.Assertion.EqualityAssertion;
import static org.sciborgs1155.lib.Assertion.eAssert;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;
import static org.sciborgs1155.robot.shooter.ShooterConstants.GEARING;
import static org.sciborgs1155.robot.vision.VisionConstants.MAX_ANGLE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.lib.Test;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;


public class Pivot {
    private final PivotIO hardware;
    private final SysIdRoutine sysId;

    private final ProfiledPIDController pid = 
    new ProfiledPIDController(
        kP, 
        kI, 
        kD, 
        new TrapezoidProfile.Constraints(MAX_VELOCITY.in(RadiansPerSecond), MAX_ACCEL.in(RadiansPerSecondPerSecond)));
    private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

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

        pid.reset(MAX_ANGLE);
        pid.setTolerance(POSITION_TOLERANCE.in(Radians));

        SmartDashboard.putData("pivot quasistatic forward", quasistaticForward());
        SmartDashboard.putData("pivot quasistatic backward", quasistaticBack());
        SmartDashboard.putData("pivot dynamic forward", dynamicForward());
        SmartDashboard.putData("pivot dynamic backward", dynamicBack());

        setDefaultCommand(
            // run(() -> update(MAX_ANGLE.in(Radians)))
            //     .until(() -> pid.getGoal().position > MAX_ANGLE.in(Radians))
            //     .andThen(run(() -> pivot.setVoltage(0)))
            //     .withName("default pos")
        );





        

    }

    public Command quasistaticForward() {}
    public Command quasistaticBack() {}
    public Command dynamicForward() {}
    public Command dynamicBack() {}
}
