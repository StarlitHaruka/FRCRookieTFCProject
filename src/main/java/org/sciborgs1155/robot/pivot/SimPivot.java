package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.pivot.PivotConstants.LENGTH;
import static org.sciborgs1155.robot.pivot.PivotConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.pivot.PivotConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.pivot.PivotConstants.MOI;
import static org.sciborgs1155.robot.pivot.PivotConstants.MOTOR_GEARING;
import static org.sciborgs1155.robot.pivot.PivotConstants.STARTING_ANGLE;

import org.sciborgs1155.robot.Constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import monologue.Logged;
import monologue.Annotations.Log;

public class SimPivot implements PivotIO, Logged{

    private final SingleJointedArmSim sim = 
    new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
              DCMotor.getNEO(4), MOI.in(KilogramSquareMeters), 1.0 / MOTOR_GEARING), 
        DCMotor.getNEO(4), 
        1 / MOTOR_GEARING, 
        -LENGTH.in(Meters), 
        MIN_ANGLE.in(Radians), 
        MAX_ANGLE.in(Radians), 
        true, 
        getPosition(), 
        STARTING_ANGLE.in(Radians));

    @Override
    public void setVoltage(double voltage) {
       sim.setInputVoltage(voltage);
       sim.update(Constants.PERIOD.in(Seconds));
    }

    @Override
    @Log.NT
    public double getPosition() {
        return sim.getAngleRads();
    }

    @Override
    public double getVelocity() {
        return sim.getVelocityRadPerSec();
    }

    /**
     * left here to fit IO but not used
     */
    @Override
    public void setCurrentLimit(Current limit) {}

    @Override
    public void close() throws Exception {}
    
}
