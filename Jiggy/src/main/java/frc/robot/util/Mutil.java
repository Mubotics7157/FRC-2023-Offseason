package frc.robot.util;

import frc.robot.Constants.DriveConstants;

public final class Mutil {
    
    /**
   * Converts controller input into chassis speeds
   * @param val controller input to modify
   * @param isRot boolean for if this is for rotation
   * @return meters per second
   */

    public static double modifyInputs(double val, boolean isRot){
        if(isRot){
            if(Math.abs(val) < 0.1){
                val = 0;
            }
            return val * DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
        }
        
        else{
            if(Math.abs(val) < 0.1){
                val = 0;
            }
            return val * DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
        }
    }

    /**
   * Multiplies input to factor with deadband
   * @param val value to modify
   * @param factor multiplies the value 
   * @return whatever your value is times the factor (as long as the absolute value of value is less than 0.1)
   */

   public static double modifyInputs(double val, double factor){
        if(Math.abs(val) < 0.1){
            val = 0;
        }
        return val * factor;
    
}
}
