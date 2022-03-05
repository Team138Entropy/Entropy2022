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

public class TEST extends AutoModeBase {
    List<TurnInPlaceAction> driveActionList = new ArrayList<TurnInPlaceAction>();  


    public TEST(){
        // add multiple actions to drive trajectorys, these will run one after another
      driveActionList.add(new TurnInPlaceAction(90));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(TurnInPlaceAction currentAction:  driveActionList )  {
        runAction(currentAction);
     }
    }
}