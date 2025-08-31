package org.sciborgs1155.robot.pivot;

public interface PivotIO {

    /**
     * sets voltage of a pivot
     * @param voltage
     */
    public void setVoltage(double voltage);

    /**
     * returns current pos of pivot
     * @return pos in rads
     */
    public double getPosition();

    /**
     * returns current velocity of pivot 
     * @return
     */
    public double getVelocity();

    /**
     * sets the current limit for pivots 
     * *there is a distinction between reg and climber pivots here i will leave for later
     *  once i understand it
     * @param limit
     */
    public void MaxCurrentLimit(double limit);

}
