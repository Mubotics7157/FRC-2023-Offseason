package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LiveNumber {

    private String entryName;
    private double defaultValue;
    
    public LiveNumber(String entryName, double defaultValue){
        this.entryName = entryName;
        this.defaultValue = defaultValue;

        SmartDashboard.putNumber(entryName, defaultValue);
    }

    public void set(double value){
        SmartDashboard.putNumber(entryName, value);
    }

    public double get(){
        return SmartDashboard.getNumber(entryName, defaultValue);
    }
}
