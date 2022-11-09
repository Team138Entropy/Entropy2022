package frc.robot.auto.modes;
import frc.robot.subsystems.Arm;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
import frc.robot.auto.TrajectoryLibrary;
import frc.robot.auto.TrajectoryGeneratorHelper;
import java.util.List;
import java.util.ArrayList;

public class TEST extends AutoModeBase {
    List<Action> driveActionList = new ArrayList<Action>();  


    public TEST(boolean ball3){

        // Score Ball 1
        registerAction(new ArmRotateAction(Arm.ArmTarget.SCORE_FRONT.degrees));
        registerAction(new WaitAction(.3));
        registerAction(new EjectAction());
        registerAction(new WaitAction(.1));

        // Drive to Ball 1 and begin pickup at the same time
        List<Action> parallelList = new ArrayList<>();
        parallelList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getPathPlannerTrajectory("George")));
        parallelList.add(new PickupAction(5));
        registerAction(new ParallelAction(parallelList));
        registerAction(new WaitAction(.3));

        // Drive Back to Score
        registerAction(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getReversedTrajectory(TrajectoryLibrary.getInstance().getPathPlannerTrajectory("George"))));

        // Score Ball 2
        registerAction(new WaitAction(.1));
        registerAction(new EjectAction());
        registerAction(new WaitAction(.3));

        if(ball3)
        {
            // Drive to Ball 3 and big pickup at the same time
            List<Action> parallelList2 = new ArrayList<>();
            parallelList2.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getPathPlannerTrajectory("george2")));
            parallelList2.add(new PickupAction(5));
            registerAction(new ParallelAction(parallelList2));
            registerAction(new WaitAction(.1));

            // Drive Back to the Hub
            registerAction(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getReversedTrajectory(TrajectoryLibrary.getInstance().getPathPlannerTrajectory("george3"))));

            // Score Ball 2
            registerAction(new WaitAction(.1));
            registerAction(new EjectAction());
            registerAction(new WaitAction(.3));
        }
      
        
    }

    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  driveActionList )  {
        runAction(currentAction);
     }
    }
}