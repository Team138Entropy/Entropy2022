package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.opencv.core.Range;
import org.photonvision.*;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;


import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Logger;



/** Add your docs here. */
public class photonVision{
  public static photonVision mInstance = null;

  static PhotonCamera camera = new PhotonCamera("camera138");

    
  


  
  //Transform2d pose = target.getCameraToTarget();
  //List<TargetCorner> corners = target.getCorners();

  public static synchronized photonVision getInstance() {
    if (mInstance == null) {
      mInstance = new photonVision();
    }
    return mInstance;
  }

  public static synchronized void findingTargets(){
    try{
    PhotonPipelineResult result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();

    // Get information from target.
    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double area = target.getArea();
    double skew = target.getSkew();
    }
    finally{
      
    }
  }

  public synchronized PhotonPipelineResult getPipeLine() {
    return camera.getLatestResult();
  }
  
  public synchronized List<PhotonTrackedTarget> getTargetList() {
    PhotonPipelineResult pipeLine = getPipeLine();
    List<PhotonTrackedTarget> targets = pipeLine.getTargets();
    return targets;
  }

  public synchronized  PhotonTrackedTarget bestTarget(){
    PhotonPipelineResult pipeLine = getPipeLine();
    return pipeLine.getBestTarget();
  }

  public synchronized double targetDist(){
    //TODO input camera values
    double CAMERA_HEIGHT_METERS = 0;
    double TARGET_HEIGHT_METERS = 0;
    double CAMERA_PITCH_RADIANS = 0;
    var result = camera.getLatestResult();
    try{
      if (result.hasTargets()) {
        // First calculate range
        double range =
          PhotonUtils.calculateDistanceToTargetMeters(
                  CAMERA_HEIGHT_METERS,
                  TARGET_HEIGHT_METERS,
                  CAMERA_PITCH_RADIANS,
                  Units.degreesToRadians(result.getBestTarget().getPitch()));
                  return range;}
      else{
        return 0.0;
      }
    }finally{}
    
    
  
  }

/*
  @Override
  public void zeroSensors() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void checkSubsystem() {
    // TODO Auto-generated method stub
    
  }
  */
}
