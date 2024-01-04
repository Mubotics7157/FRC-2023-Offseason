package frc.robot.util;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.LimelightHelpers.LimelightResults;

public class LimelightNT4 {
    
    private String name;

    private double visionLatency = 0;

    private DoubleSubscriber tv, tx, ty, pipelineGetter;
    private DoublePublisher pipelineSetter;
    private DoublePublisher ledMode;
    private DoubleArraySubscriber bluePose, redPose;

    public LimelightNT4(String name){
        this.name = name;   

        NetworkTableInstance instance = NetworkTableInstance.getDefault();

        NetworkTable table = instance.getTable(name);

        tv = table.getDoubleTopic("tv").subscribe(0);
        tx = table.getDoubleTopic("tx").subscribe(0);
        ty = table.getDoubleTopic("ty").subscribe(0);
        pipelineGetter = table.getDoubleTopic("pipeline").subscribe(0);

        pipelineSetter = table.getDoubleTopic("pipeline").publish();
        ledMode = table.getDoubleTopic("ledMode").publish();

        bluePose = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[6]);
        redPose = table.getDoubleArrayTopic("botpose_wpired").subscribe(new double[6]);
    }

    public boolean hasTargets(){
        return tv.get() == 1;
    }

    public double getTargets(){
        return tv.get();
    }

    public Rotation2d getTargetYaw(){
        if(hasTargets())
            return Rotation2d.fromDegrees(tx.get());
        else
            throw new NullPointerException();
    }

    public Rotation2d getTargetPitch(){
        if(hasTargets())
            return Rotation2d.fromDegrees(ty.get());
        else
            throw new NullPointerException();
    }

    public Pose2d getBotPose(){
        double[] poseEntry;

        if(DriverStation.getAlliance() == Alliance.Red)
            poseEntry = redPose.get();
        else
            poseEntry = bluePose.get();

        Pose2d pose = new Pose2d(poseEntry[0], poseEntry[1], Rotation2d.fromDegrees(poseEntry[5]));
        visionLatency = poseEntry[6];
        
        return pose;
    }

    public double getLatency(){
        return visionLatency * .001;
    }

    public LimelightResults getJsonDump(){
        return LimelightHelpers.getLatestResults(name);
    }

    public double getBootTimeStamp(){
        return getJsonDump().targetingResults.timestamp_LIMELIGHT_publish;
    }

    public String getName(){
        return name;
    }

    public void setPipelineIndex(int on){
        pipelineSetter.set(on);
    }

    public int getPipelineIndex(){
        return (int)pipelineGetter.get();
    }

    public void setLEDS(boolean on){
        if(on)
            ledMode.set(3);
        else   
            ledMode.set(1);
    }


}
