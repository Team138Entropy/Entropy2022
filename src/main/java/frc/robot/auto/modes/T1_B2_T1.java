package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
import frc.robot.auto.TrajectoryLibrary;

import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import java.util.List;
import java.util.ArrayList;

public class T1_B2_T1 extends AutoModeBase {
    List<DriveTrajectoryAction > driveActionList = new ArrayList<DriveTrajectoryAction >();  


    public T1_B2_T1(){
        // add multiple actions to drive trajectorys, these will run one after another
        driveActionList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_New_T1_B2()));
        driveActionList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getReversedTrajectory(TrajectoryLibrary.getInstance().get_New_T1_B2())));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(DriveTrajectoryAction currentAction:  driveActionList )  {
        runAction(currentAction);
     }
    }
}