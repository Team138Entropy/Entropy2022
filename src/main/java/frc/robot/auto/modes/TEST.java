package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
import frc.robot.auto.TrajectoryLibrary;
import frc.robot.auto.TrajectoryGeneratorHelper;
import java.util.List;
import java.util.ArrayList;

public class TEST extends AutoModeBase {
    List<Action> driveActionList = new ArrayList<Action>();  


    public TEST(){
        // add multiple actions to drive trajectorys, these will run one after another
        //driveActionList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getReversedTrajectory(TrajectoryLibrary.getInstance().get_C_Turn())));
        // driveActionList.add(new StoreDrivePositionAction());
        // driveActionList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getReversedTrajectory(TrajectoryLibrary.getInstance().get_C_Turn())));
        // driveActionList.add(new WaitAction(2));
        // driveActionList.add(new DriveGeneratedAction(false));
        driveActionList.add(new DriveTrajectoryAction(TrajectoryGeneratorHelper.getStraightTrajectory(5)));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  driveActionList )  {
        runAction(currentAction);
     }
    }
}