package org.sciborgs1155.robot.intake;

public class NoIntake implements IntakeIO{

    @Override
    public void close() throws Exception {}

    @Override
    public void setPower(double percentage) {}

    @Override
    public boolean beambreak() {
        return false;
    }

    @Override
    public double current() {
        return 0;
    }

    @Override
    public boolean seenNote() {
        return false;
    }
    
}
