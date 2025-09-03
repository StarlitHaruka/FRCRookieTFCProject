package org.sciborgs1155.robot.feeder;



public class NoFeeder implements FeederIO{

    @Override
    public void setPower(double power) {}

    @Override
    public double current() {
        return 0;
    }

    @Override
    public boolean beambreak() {
       return true;
    }
    
    @Override
    public void close() throws Exception {}

}
