package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
import frc.robot.auto.TrajectoryLibrary;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import java.util.ArrayList;

public class Tarmac1_B2_B3_Tarmac2 extends AutoModeBase {
    List<DriveTrajectoryAction > driveActionList = new ArrayList<DriveTrajectoryAction >();  


    public Tarmac1_B2_B3_Tarmac2(){
        // add multiple actions to drive trajectorys, these will run one after another
      driveActionList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_Tarmac1_B2_trajectory()));
      driveActionList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_B2_B3_trajectory()));
      driveActionList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_B3_Tarmac2_trajectory())); 
       /* dta = new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getBasicTrajectory());
        Tarmac1_B2 = new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_Tarmac1_B2_trajectory())
        B2_B3 = new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_B2_B3_trajectory())
        B3_Tarmac2 = new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_B3_Tarmac2_trajectory())
        */
    }

    @Override
    protected void routine() throws AutoModeEndedException {
       /* runAction(Tarmac1_B2);
        runAction(B2_B3);
        runAction(B3_Tarmac2);
        */
         // Traverse list and run each action
      for(DriveTrajectoryAction currentAction:  driveActionList )  {
        runAction(currentAction);
     }
    }
}