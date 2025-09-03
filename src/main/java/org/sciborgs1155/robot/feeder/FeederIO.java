package org.sciborgs1155.robot.feeder;


public interface FeederIO extends AutoCloseable{

    void setPower(double power);

    double current();

    boolean beambreak();
    
}
