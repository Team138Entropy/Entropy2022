package frc.robot.vision;

import frc.robot.Constants;
import frc.robot.util.geometry.*;
import edu.wpi.first.wpilibj.Timer;


// TargetInfo describes a Target
// A target could be a game piece, a reflective tape
// It is what the vision system is trying to track
public class TargetInfo {
    private boolean mValid;
    private double mTimestamp;
    private double mTargetID;
    private double mX;
    private double mY;
    private double mZ;
    private double mDistance;

    TargetInfo(
        double targetID,
        double y,
        double z,
        double distance
    ){
        mValid = false;
        mTargetID = targetID;
        mX = 1;
        mY = y;
        mZ = z;
        mDistance = distance;

        // set the timestamp of the packet
        updateToCurrentTimestamp();
    }

    // Constructor for Invalid Target
    TargetInfo(){
        mValid = false;
    }


    public boolean isValid(){
        return mValid;
    }

    public double getTimestamp(){
        return mTimestamp;
    }

    public void setTimestamp(double ts){
        mTimestamp = ts;
    }

    // sets the timestamp to the current timestamp
    public void updateToCurrentTimestamp(){
        mTimestamp = Timer.getFPGATimestamp();
    }

    // angle to target
    public double getErrorAngle(){
            // Compensate for camera pitch
        Translation2d xz_plane_translation =
            new Translation2d(mX, mZ).rotateBy(Constants.Vision.kCameraHorizontalPlaneToLens);
        double x = xz_plane_translation.x();
        double y = mY;
        double z = xz_plane_translation.y();

        double distance = getDistance();
        distance = 1;
        Rotation2d angle = new Rotation2d(x, y, true);
        Translation2d td = angle.toTranslation();
        td.translateBy(new Translation2d(0, 10));
    
        // System.out.println("Camera's Angle to Vision Target: " + angle.getDegrees());
        Translation2d t = new Translation2d(distance * angle.cos(), distance * angle.sin());
        t.StoreDistance = getDistance();
        angle = td.direction();
        return angle.getDegrees()-4;
    }

    public void CalculateFields(){
        mValid = true;
        double nY = 0;
        double nZ = 0;
        // 320x240 - ball camera
        // 640x480 - turret camera
    
        // packets from High Goal Camera
        // Height (z), horizontal (y)
        nZ = (1.0 / 240.0) * (239.5 - mZ);
        nY = (1.0 / 320.0) * (mY - 319.5);

        // nY = -((y - 320.0)/320.0);
        // nZ = -((z - 240.0)/240.0);
        mY = (Constants.Vision.horizontalView / 2) * nY;
        mZ = (Constants.Vision.verticalView / 2) * nZ;
     
        
    }

    public double getDistance(){
        return mDistance;
    }

}
