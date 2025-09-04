package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.intake.IntakeConstants.DEBOUNCE_TIME;
import static org.sciborgs1155.robot.intake.IntakeConstants.INTAKE_SPEED;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.commands.NoteVisualizer;

public class Intake extends SubsystemBase implements AutoCloseable, Logged{

    public static Intake create() {
        return Robot.isReal() ? new Intake(new RealIntake()) : new Intake(new NoIntake());
    }

    public static Intake none() {
        return new Intake(new NoIntake());
    }

    private final IntakeIO hardware;
    private final EventLoop intakeTriggerPoller = new EventLoop();
    private final Trigger intakeTrigger;

    public Intake(IntakeIO hardware) {
        this.hardware = hardware;

        intakeTrigger = new Trigger(intakeTriggerPoller, hardware::seenNote);
        intakeTrigger.onFalse(stop());

    }

    public Command intake() {
        return Commands.waitUntil(hasNote())
        .andThen(Commands.waitUntil(hasNote().negate()))
        .alongWith(NoteVisualizer.intake())
        .withName("intake");

    }

    public Command runIntake(double power) {
        return runOnce(() -> hardware.setPower(power))
        .andThen(Commands.idle(this))
        .finallyDo(() -> hardware.setPower(0));

    }

    public Command forward() {
        return runIntake(INTAKE_SPEED).withName("forward");
    }

    public Command backward() {
        return runIntake(-INTAKE_SPEED).withName("backward");
    }

    public Command stop() {
        return runIntake(0);
    }

    public Trigger hasNote() {
        return new Trigger(hardware::beambreak)
        .negate()
        .debounce(DEBOUNCE_TIME.in(Seconds), DebounceType.kFalling);
    }

    @Log.NT
    public boolean stall() {
        return hardware.current() > DCMotor.getNeoVortex(1).stallCurrentAmps;
    }

    public void pollTrigger () {
        intakeTriggerPoller.poll();
    }

    public void periodic() {
        log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
    }

    @Override
    public void close() throws Exception {
        hardware.close();
    }
    
}
