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
        driveActionList.add(new ArmRotateAction(Arm.ArmTarget.SCORE_FRONT.degrees));
        driveActionList.add(new WaitAction(.5));
        driveActionList.add(new EjectAction());
        driveActionList.add(new WaitAction(.5));
        }
        
        driveActionList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getReversedTrajectory(TrajectoryLibrary.getInstance().get_New_T35_B5_mod())));
        driveActionList.add(new ArmRotateAction(Arm.ArmTarget.HOME.degrees));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  driveActionList )  {
        runAction(currentAction);
     }
    }
}