package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.Arrays;
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

  public static boolean hasTarget = false;
  
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
    try{
      System.out.println("calling getPipeLine");
      return camera.getLatestResult();
    }finally{

    }
    
  }

  public synchronized static double getTargetYaw(){
    PhotonTrackedTarget myTarget = null;
      var result = camera.getLatestResult();
      double targetYaw = 0.0;
      try{
        myTarget = result.getBestTarget();
      }finally{}

      if(myTarget != null){
        targetYaw = myTarget.getYaw();
        //System.out.println("target yaw:" + targetYaw);
        hasTarget = true;
      }

      return targetYaw;
    
  }
  
  public synchronized List<PhotonTrackedTarget> getTargetList() {
    try{
      System.out.println("calling getTargetList");
      PhotonPipelineResult pipeLine = getPipeLine();
      List<PhotonTrackedTarget> targets = pipeLine.getTargets();

      //System.out.println(targets);
      return targets;
    }finally{}
  }


  public synchronized  PhotonTrackedTarget bestTarget(){
    System.out.println("calling bestTarget");
    PhotonPipelineResult pipeLine = getPipeLine();
    return pipeLine.getBestTarget();
  }

  public synchronized double targetDist(){
    try{
      System.out.println("calling targetDist");
      //TODO input camera values
      double CAMERA_HEIGHT_METERS = 1;
      double TARGET_HEIGHT_METERS = 0.5;
      double CAMERA_PITCH_RADIANS = 1;
      double range = 0;
      var result = camera.getLatestResult();
      //System.out.println(result.getBestTarget().getYaw());
      /*
      System.out.println("result:" + result);
      PhotonTrackedTarget myBestTarget = result.getBestTarget();
      System.out.println("Yaw: " + myBestTarget.getYaw());
      */

      try{
        if (result.hasTargets()) {

          // First calculate range
          range =
            PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS,
                    TARGET_HEIGHT_METERS,
                    CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
                    
                    System.out.println("Range:"+range);
                    return range;}
        
        else{
          System.out.println("returning 0.0");
          return 0.0;
        }
        

      }finally{}
    }finally{

    }
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
