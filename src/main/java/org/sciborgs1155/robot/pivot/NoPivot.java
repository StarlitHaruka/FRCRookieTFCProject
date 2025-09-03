package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Current;

public class NoPivot implements PivotIO{

    @Override
    public void setVoltage(double voltage) {}

    @Override
    public double getPosition() {
        return PivotConstants.MAX_ANGLE.in(Radians);
    }

    @Override
    public double getVelocity() {
        return 0;
    }

    @Override
    public void setCurrentLimit(Current limit) {}

    @Override
    public void close() throws Exception {}
    
    
}
