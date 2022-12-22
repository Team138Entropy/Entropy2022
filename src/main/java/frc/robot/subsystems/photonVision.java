package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.Arrays;
import java.util.List;
import java.util.regex.*;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.opencv.core.Range;
import org.opencv.photo.Photo;
import org.photonvision.*;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Logger;



/** Add your docs here. */
public class photonVision{
  public static photonVision mInstance = null;

  static PhotonCamera camera = new PhotonCamera("camera138");

  public boolean hasTarget = false;
  
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
      //System.out.println("calling getPipeLine");
      return camera.getLatestResult();
    }finally{

    }
    
  }

  public synchronized String getTargetID() {
    //System.out.println("calling getTargetID");
    String myIDString = "0";
    int myIDInt = 0;
    List<PhotonTrackedTarget> myItems = getTargetList();
    String stringTargetList = myItems.toString();
    Pattern pattern = Pattern.compile("(?<=fiducialId=)(.*)(?=, cam)", Pattern.CASE_INSENSITIVE);
    Matcher matcher = pattern.matcher(stringTargetList);
    while (matcher.find()){
      myIDString =  matcher.group(1);
      //System.out.println(myIDString);
    }
    /*
    try{
      System.out.println("before string to int");
      System.out.println(myIDString);
      myIDInt= Integer.valueOf(myIDString);
      System.out.println("after string to int");
    }finally{
      System.out.println("connot convert string to int");
    }
    */
    
    return myIDString;
  }

  public synchronized boolean seesTargets(){
    var result = camera.getLatestResult();
    boolean haveATarget = result.hasTargets();
    
    return haveATarget;
  }

  public synchronized double getTargetYaw(){

    double targetYaw = Double.NaN;
    PhotonTrackedTarget myTarget = null;

    var result = camera.getLatestResult();
    boolean seesTargets = seesTargets();
    //System.out.println("sees targets: " + seesTargets);
    
    if(seesTargets){
      
      myTarget = result.getBestTarget();
      targetYaw = myTarget.getYaw();
    }
    else{
      targetYaw = 0;
      
    }
      
    return targetYaw;
  }
  
  public synchronized List<PhotonTrackedTarget> getTargetList() {
    try{
      //System.out.println("calling getTargetList");
      PhotonPipelineResult pipeLine = getPipeLine();
      List<PhotonTrackedTarget> targets = pipeLine.getTargets();

      //System.out.println(targets);
      return targets;
    }finally{}
  }


  public synchronized  PhotonTrackedTarget bestTarget(){
    //options for deciding what target is best (for later)
    /*
    Largest, Smallest, Highest (towards the top of the image) , Lowest , Rightmost (Best target on the right, worst on left), Leftmost, Centermost
    */
    //System.out.println("calling bestTarget");
    PhotonPipelineResult pipeLine = getPipeLine();
    return pipeLine.getBestTarget();
  }

  public synchronized double targetDist(){
    try{
      //System.out.println("calling targetDist");
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
                    
                    //System.out.println("Range:"+range);
                    return range;}
        
        else{
          //System.out.println("returning 0.0");
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
