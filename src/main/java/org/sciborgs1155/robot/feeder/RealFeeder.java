package org.sciborgs1155.robot.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.lib.FaultLogger.*;
import static org.sciborgs1155.robot.Ports.Feeder.*;
import static org.sciborgs1155.robot.feeder.FeederConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.SparkUtils;

public class RealFeeder implements FeederIO{

    private final SparkFlex motor;
    private final SparkFlexConfig config = new SparkFlexConfig();
    private final DigitalInput beambreak;

    public RealFeeder() {
        motor = new SparkFlex(FEEDER_SPARK, MotorType.kBrushless);

        check(
            motor,
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters)
        );

        config.apply(SparkUtils.getStatusConfigurationOfNothingFrameStrategy());
        config.apply(config.idleMode(IdleMode.kBrake).smartCurrentLimit((int)CURRENT_LIMIT.in(Amps)));
        config.apply(config.openLoopRampRate(RAMP_TIME.in(Seconds)));

        beambreak = new DigitalInput(BEAMBREAK);

        register(motor);
    }


    @Override
    public void close() throws Exception {
        motor.close();
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
        check(motor);
    }

    @Override
    @Log.NT
    public double current() {
        return motor.getOutputCurrent();
    }

    @Override
    public boolean beambreak() {
        return beambreak.get();
    }

    
    
}
