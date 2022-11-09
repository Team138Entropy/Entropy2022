package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
import frc.robot.subsystems.Arm;
import frc.robot.auto.TrajectoryLibrary;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import java.util.ArrayList;

public class OneBall extends AutoModeBase {
    List<Action > driveActionList = new ArrayList<Action >();  


    public OneBall(boolean eject){
        // add multiple actions to drive trajectorys, these will run one after another
        //driveActionList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_New_T1_B2()));
        if (eject){
        registerAction(new ArmRotateAction(Arm.ArmTarget.SCORE_FRONT.degrees));
        registerAction(new WaitAction(.5));
        registerAction(new EjectAction());
        registerAction(new WaitAction(.5));

        }
        
        registerAction(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getPathPlannerTrajectory("Taxi")));
        registerAction(new ArmRotateAction(Arm.ArmTarget.HOME.degrees));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  driveActionList )  {
        runAction(currentAction);
     }
    }
}