package org.sciborgs1155.robot.pivot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static org.sciborgs1155.robot.Ports.Pivot.*;
import static org.sciborgs1155.robot.pivot.PivotConstants.*;

public class RealPivot implements PivotIO{

    private final SparkMax lead;
    private final SparkMax leftBottom;
    private final SparkMax rightBottom;
    private final SparkMax rightTop;

    leaderConfig = new SparkMaxConfig();
    followerConfig = new SparkMaxConfig();
    invertedConfig = new SparkMaxConfig();

    public RealPivot() {

        leftBottom = new SparkMax(SPARK_LEFT_BOTTOM, MotorType.kBrushless);
        lead = new SparkMax(SPARK_LEFT_TOP, MotorType.kBrushless);
        rightBottom = new SparkMax(SPARK_RIGHT_BOTTOM, MotorType.kBrushless);
        rightTop = new SparkMax(SPARK_RIGHT_TOP, MotorType.kBrushless);

    

    }

    @Override
    public void setVoltage(double voltage) {
        lead.setVoltage(voltage);

    }

    @Override
    public double getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public double getVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
    }

    @Override
    public void MaxCurrentLimit(double limit) {
        //seems to be a deprecated method..
    }
    
}
