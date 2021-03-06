package frc.robot.auto.modes;
import frc.robot.subsystems.Arm;

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

public class T35_B5 extends AutoModeBase {
    List<Action > driveActionList = new ArrayList<Action >();  
    

    public T35_B5(){
        // add multiple actions to drive trajectorys, these will run one after another
        /*
        driveActionList.add(new ArmRotateAction(Arm.ArmTarget.SCORE_FRONT.degrees));
        //driveActionList.add(new WaitAction(.3));
        driveActionList.add(new EjectAction());
        //driveActionList.add(new WaitAction(.3));
        //driveActionList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getReversedTrajectory(TrajectoryLibrary.getInstance().get_New_T35_B5())));
        //driveActionList.add(new ArmRotateAction(Arm.ArmTarget.INTAKE.degrees));
        driveActionList.add(new AutoSteerAction(false));
        driveActionList.add(new ArmRotateAction(Arm.ArmTarget.SCORE_FRONT.degrees));
        driveActionList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_New_T35_B5()));
        driveActionList.add(new ArmRotateAction(Arm.ArmTarget.SCORE_FRONT.degrees));
        driveActionList.add(new EjectAction());
        */
        /*
        List<Action > driveBackActions = new ArrayList<Action >(); 
        driveBackActions.add(new ArmRotateAction(Arm.ArmTarget.SCORE_FRONT.degrees));
        driveBackActions.add(new GrasperUpdateAction(3));
        driveBackActions.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_New_T35_B5()));
        */
        driveActionList.add(new ArmRotateAction(Arm.ArmTarget.SCORE_FRONT.degrees));
        driveActionList.add(new EjectAction());
        driveActionList.add(new StoreDrivePositionAction());
        driveActionList.add(new AutoTurnAction());
        driveActionList.add(new DriveUntilPickupAction());

        List<Action> parallelList = new ArrayList<>();
        parallelList.add(new ArmRotateAction(Arm.ArmTarget.SCORE_FRONT.degrees));
        parallelList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_New_T35_B5()));

        driveActionList.add(new ParallelAction(parallelList));
        
        //driveActionList.add(new DriveGeneratedAction(true));
        driveActionList.add(new EjectAction());

        //driveActionList.add(new ParallelAction(driveBackActions));
        //driveActionList.add(new EjectAction());
    }

    @Override
    protected void routine() throws AutoModeEndedException {
         // Traverse list and run each action
      for(Action currentAction:  driveActionList )  {
        runAction(currentAction);
     }
    }
}