package org.sciborgs1155.robot.pivot;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class PivotVisualizer implements Sendable, AutoCloseable {

    private final Mechanism2d mech;
    private final MechanismLigament2d arm; 

    public PivotVisualizer(Color8Bit color) {
        mech = new Mechanism2d(2, 2);
        arm = new MechanismLigament2d("arm", LENGTH.in(Meters), 0, 4, color);
        MechanismRoot2d chassis = mech.getRoot("chassis", 1 + AXLE_FROM_CHASSIS.getX(), AXLE_FROM_CHASSIS.getZ());
    }

    public void setState(double angle) {
        arm.setAngle(Units.radiansToDegrees(angle + Math.PI));
    }

    @Override
    public void close() throws Exception {
        mech.close();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        mech.initSendable(builder);
    }


}
