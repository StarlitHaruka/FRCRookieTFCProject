package org.sciborgs1155.robot.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.lib.FaultLogger.check;
import static org.sciborgs1155.lib.FaultLogger.register;
import static org.sciborgs1155.robot.intake.IntakeConstants.CURRENT_LIMIT;
import static org.sciborgs1155.robot.intake.IntakeConstants.RAMP_TIME;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.robot.Ports;

public class RealIntake implements IntakeIO{

    private final SparkFlex spark = new SparkFlex(Ports.Intake.INTAKE_SPARK, MotorType.kBrushless);
    private final SparkFlexConfig config = new SparkFlexConfig();
    private final DigitalInput beambreak = new DigitalInput(Ports.Intake.BEAMBREAK);
    private final AsynchronousInterrupt asynchronousInterrupt;
    private boolean noteDetected = false;

    public RealIntake() {

        check(spark, spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
        config.apply(SparkUtils.getStatusConfigurationOfNothingFrameStrategy());
        config.apply(config.inverted(true));
        config.apply(config.idleMode(IdleMode.kBrake).smartCurrentLimit((int) CURRENT_LIMIT.in(Amps)));
        config.apply(config.openLoopRampRate(RAMP_TIME.in(Seconds)));

        check(spark, spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));

        register(spark);

        asynchronousInterrupt = 
            new AsynchronousInterrupt(
                beambreak,
                (Boolean rising, Boolean falling) -> {
                    if (falling) {
                        noteDetected = true;
                    } else if (rising) {
                        noteDetected = false;
                    }


                });

        asynchronousInterrupt.setInterruptEdges(true, true);
        asynchronousInterrupt.enable();
    }

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }

    @Override
    public void setPower(double percentage) {
        spark.set(percentage);
        check(spark);

    }

    @Override
    public boolean beambreak() {
        return beambreak.get();
    }

    @Override
    public double current() {
        return spark.getOutputCurrent();
    }

    @Override
    @Log.NT
    public boolean seenNote() {
        return noteDetected;
    }
    
}
