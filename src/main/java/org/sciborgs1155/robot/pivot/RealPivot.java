package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static org.sciborgs1155.lib.FaultLogger.*;
import static org.sciborgs1155.robot.Ports.Pivot.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Current;
import java.util.Set;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealPivot implements PivotIO{

    private final SparkMax lead;
    private final SparkMax leftBottom;
    private final SparkMax rightBottom;
    private final SparkMax rightTop;
    private final RelativeEncoder encoder, integratedEncoder;
    private final SparkMaxConfig leaderConfig, followerConfig, invConfig;



    public RealPivot() {

        leftBottom = new SparkMax(SPARK_LEFT_BOTTOM, MotorType.kBrushless);
        lead = new SparkMax(SPARK_LEFT_TOP, MotorType.kBrushless);
        rightBottom = new SparkMax(SPARK_RIGHT_BOTTOM, MotorType.kBrushless);
        rightTop = new SparkMax(SPARK_RIGHT_TOP, MotorType.kBrushless);

        leaderConfig = new SparkMaxConfig();
        followerConfig = new SparkMaxConfig();
        invConfig = new SparkMaxConfig();

        integratedEncoder = lead.getEncoder();
        encoder = leftBottom.getAlternateEncoder();

        check(
                lead,
                lead.configure(
                    leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
            leaderConfig.apply(
                SparkUtils.getSignalsConfigurationFrameStrategy(
                    Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
                    Set.of(Sensor.ALTERNATE, Sensor.INTEGRATED),
                    false));

        leaderConfig.apply(leaderConfig.inverted(true));
        leaderConfig.apply(leaderConfig.idleMode(IdleMode.kBrake));

        leaderConfig.apply(
        leaderConfig
            .alternateEncoder
            .inverted(true)
            .positionConversionFactor(POSITION_FACTOR.in(Radians) / 2.0)
            .velocityConversionFactor(VELOCITY_FACTOR.in(RadiansPerSecond) / 2.0));

        leaderConfig.apply(
        leaderConfig
            .encoder
            .positionConversionFactor(Rotations.of(MOTOR_GEARING).in(Radians))
            .velocityConversionFactor(
                Rotations.of(MOTOR_GEARING).per(Minute).in(RadiansPerSecond)));

        check(lead, encoder.setPosition(STARTING_ANGLE.in(Radians)));

        leaderConfig.apply(leaderConfig.softLimit.reverseSoftLimit((float) MIN_ANGLE.in(Radians)));

        leaderConfig.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));

        check(
            lead,
            lead.configure(
                leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));

        followerConfig.apply(leaderConfig);
        followerConfig.apply(
            followerConfig
                .follow(lead)
                .apply(SparkUtils.getStatusConfigurationOfNothingFrameStrategy()));
        invConfig.apply(leaderConfig);
        invConfig.apply(
            invConfig
                .follow(lead, true)
                .apply(SparkUtils.getStatusConfigurationOfNothingFrameStrategy()));

        /* 
        * check for errors
        */
        check(
            leftBottom,
            leftBottom.configure(
                followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
        check(
            rightBottom,
            rightBottom.configure(
                invConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
        check(
            rightTop,
            rightTop.configure(
                invConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));

        register(lead);
        register(leftBottom);
        register(rightBottom);
        register(rightTop);

        /**
         * working maxcurrent
         * draws from variable CURRENT_LIMIT 
         */
        leaderConfig.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));

    }

    @Override
    public void setVoltage(double voltage) {
        lead.setVoltage(voltage);

    }

    @Override
    @Log.NT
    public double getPosition() {
        return integratedEncoder.getPosition() + STARTING_ANGLE.in(Radians);
    }

    @Override
    public double getVelocity() {
        return integratedEncoder.getVelocity();
    }

    @Override
    public void setCurrentLimit(Current limit) {
        //seems to be a deprecated method replaced by a line up top..
    }

    @Override
    public void close() throws Exception {
     lead.close();
     leftBottom.close();
     rightBottom.close();
     rightTop.close();
    }
    
}
