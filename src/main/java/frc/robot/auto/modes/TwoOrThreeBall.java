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

 public class TwoOrThreeBall extends AutoModeBase {
    List<Action > AutoActionList = new ArrayList<Action >();  

    // If Argument is false, only 2 Balls
    // if Argument is true, 3 balls
    public TwoOrThreeBall(boolean threeBall){
        // Score Ball 1
        AutoActionList.add(new ArmRotateAction(Arm.ArmTarget.SCORE_FRONT.degrees));
        AutoActionList.add(new WaitAction(.35));
        AutoActionList.add(new EjectAction());
        AutoActionList.add(new StoreDrivePositionAction());
        AutoActionList.add(new AutoTurnAction()); // Aim for Ball 2
        AutoActionList.add(new DriveUntilPickupAction());

        List<Action> Ball2Score = new ArrayList<>();
        Ball2Score.add(new ArmRotateAction(Arm.ArmTarget.SCORE_FRONT.degrees));
        //Ball2Score.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_New_T35_B5())); //old style
        Ball2Score.add(new DriveGeneratedAction(false));

        AutoActionList.add(new ParallelAction(Ball2Score));
        AutoActionList.add(new EjectAction());
        AutoActionList.add(new WaitAction(.15));

 
        if(threeBall){
            // 3 Balls
            AutoActionList.add(new StoreDrivePositionAction());
            // C Turn to get in range of next ball
            AutoActionList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getReversedTrajectory(TrajectoryLibrary.getInstance().get_C_Turn())));
            AutoActionList.add(new AutoTurnAction()); // Aim for Ball 3
            AutoActionList.add(new DriveUntilPickupAction());

            // got the ball, now go back to score it
            List<Action> Ball3Score = new ArrayList<>();
            Ball3Score.add(new ArmRotateAction(Arm.ArmTarget.SCORE_FRONT.degrees));
            //Ball3Score.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_New_T35_B5())); //old style
            Ball3Score.add(new DriveGeneratedAction(false));
            AutoActionList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_C_Turn()));

            AutoActionList.add(new ParallelAction(Ball3Score));
            AutoActionList.add(new EjectAction());
        }else{
            // 2 Balls
            AutoActionList.add(new ArmRotateAction(Arm.ArmTarget.HOME.degrees));
            //maybe drive out of tarmac?
        }
    }

    @Override
    protected void routine() throws AutoModeEndedException {
      // Traverse list and run each action
      for(Action currentAction:  AutoActionList )  {
        runAction(currentAction);
     }
    }

}