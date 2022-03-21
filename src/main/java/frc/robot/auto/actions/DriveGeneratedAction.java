package frc.robot.auto.actions;

import frc.robot.subsystems.Drive;
import frc.robot.auto.TrajectoryFollower;
import frc.robot.auto.TrajectoryLibrary;
import edu.wpi.first.math.trajectory.*;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;

/**
 * Stores the Initial Drive Position Action
 * Useful for generating a backtracked position
 */
public class DriveGeneratedAction implements Action {
  private final Drive mDrive  = Drive.getInstance();
  private boolean mReverse;
  private boolean mStopWhenDone;
  private Trajectory mTrajectory;
  private TrajectoryFollower mTrajectoryFollower = TrajectoryFollower.getInstance();
  private TrajectoryLibrary mTrajectoryLibrary = TrajectoryLibrary.getInstance();


  public DriveGeneratedAction(boolean reverse, boolean stopWhenDone) {
    mStopWhenDone = true;
    mReverse = reverse;
  }

  public DriveGeneratedAction(boolean reverse){
    this(reverse, true);
  }

  @Override
  public void start() {
    // generate trajectory
    Pose2d StartPose = mDrive.getStoredPose();
    Pose2d EndPose = mDrive.getPose();

    System.out.println("Generating Drive Action");
    System.out.println("Starting Pose: " + StartPose.toString());
    System.out.println("Ending Pose: " + EndPose.toString());

    // Generate trajectory
    TrajectoryConfig trajConfig = new TrajectoryConfig(Constants.Drive.Auto.MaxVelocityMetersPerSecond, 
                                  Constants.Drive.Auto.MaxAccelerationMetersPerSecondSq);
    mTrajectory = TrajectoryGenerator.generateTrajectory(StartPose, new ArrayList<Translation2d>(), 
                                                                    EndPose, trajConfig);

    // reverse trajectory (if applicable)
    if(mReverse) mTrajectory = mTrajectoryLibrary.getReversedTrajectory(mTrajectory);

    // set Trajectory into TrajectoryFollower
    mTrajectoryFollower.setTrajectory(mTrajectory);

    // perform intailization of trajectory follower
    mTrajectoryFollower.Start();
  }

  @Override
  public void update() {
    // update trajectory
    mTrajectoryFollower.Update();
  }

  @Override
  public boolean isFinished() {
    return mTrajectoryFollower.isComplete();
  }

  @Override
  public void done() {
    System.out.println("DriveGeneratedAction Complete!");
    if (mStopWhenDone) {
      System.out.println("Stop!");
        mTrajectoryFollower.StopDrive();
    }
  }
}
