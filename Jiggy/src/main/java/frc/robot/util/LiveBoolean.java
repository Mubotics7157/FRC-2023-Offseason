package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LiveBoolean {

    private String entryName;
    private boolean defaultValue;
    
    public LiveBoolean(String entryName, Boolean defaultValue){
        this.entryName = entryName;
        this.defaultValue = defaultValue;

        SmartDashboard.putBoolean(entryName, defaultValue);
    }

    public void set(boolean value){
        SmartDashboard.putBoolean(entryName, value);
    }

    public boolean get(){
        return SmartDashboard.getBoolean(entryName, defaultValue);
    }
}
