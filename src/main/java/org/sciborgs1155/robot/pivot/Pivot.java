package org.sciborgs1155.robot.pivot;

import org.sciborgs1155.robot.Robot;


public class Pivot {
    private final PivotIO hardware;

    public static Pivot create() {
        return Robot.isReal() ? 
        new Pivot(new RealPivot()) 
        : new Pivot(new SimPivot());
    }

    public static Pivot none() {
        return new Pivot(new NoPivot());
      }

    public Pivot(PivotIO pivot) {
        this.hardware = pivot;

    }
    
}
