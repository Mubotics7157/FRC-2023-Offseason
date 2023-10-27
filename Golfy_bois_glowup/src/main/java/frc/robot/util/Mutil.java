package frc.robot.util;

import frc.robot.Constants.DriveConstants;

public final class Mutil {
    
    /**
   * Converts controller input into chassis speeds
   * @param val controller input to 
   * @param isRot boolean for if this is for rotation
   * @return meters per second
   */

    public static double modifyInputs(double val, boolean isRot){
        if(isRot){
            if(Math.abs(val) < 0.1){
                val = 0;
            }
            //return val*DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
            //val = Math.copySign(Math.pow(val, 2),val);
            return val * DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
        }
        else{
            if(Math.abs(val) < 0.1){
                val = 0;
            }
            //return val*DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
            //val = Math.copySign(Math.pow(val, 2),val);
            return val * DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
        }
    }
}
