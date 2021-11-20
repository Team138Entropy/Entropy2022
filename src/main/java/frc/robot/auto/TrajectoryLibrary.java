package frc.robot.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;


// Functions for Trajectories
public class TrajectoryLibrary {
    private static TrajectoryLibrary mInstance;

    public static synchronized TrajectoryLibrary getInstance() {
        if (mInstance == null) {
          mInstance = new TrajectoryLibrary();
        }
        return mInstance;
     }
    
     private TrajectoryLibrary(){
    
    
     }
    
}


