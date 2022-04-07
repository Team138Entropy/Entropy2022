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
import frc.robot.auto.TrajectoryGeneratorHelper;
import java.util.List;
import java.util.ArrayList;

 public class TwoOrThreeBall extends AutoModeBase {
    List<Action > AutoActionList = new ArrayList<Action >();  

    // If Argument is false, only 2 Balls
    // if Argument is true, 3 balls
    public TwoOrThreeBall(boolean threeBall){
        //AutoActionList.add(new TurnInPlaceAction(45));
        
        // Score Ball 1
        AutoActionList.add(new ArmRotateAction(Arm.ArmTarget.SCORE_FRONT.degrees));
        if (!threeBall){ //extra timing for 2 ball.. just in case
            AutoActionList.add(new WaitAction(.20));
        }
        AutoActionList.add(new WaitAction(.28));
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
            // C Turn to get in range of next ball
            //AutoActionList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getReversedTrajectory(TrajectoryLibrary.getInstance().get_C_Turn())));

            // drive straight and move arm
            List<Action> DriveAndMoveArm = new ArrayList();
            DriveAndMoveArm.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getReversedTrajectory(TrajectoryGeneratorHelper.getStraightTrajectory(.35))));
            DriveAndMoveArm.add(new ArmRotateAction(Arm.ArmTarget.PRE_INTAKE.degrees)); //put arm just 
            
            
            
            AutoActionList.add(new ParallelAction(DriveAndMoveArm));
            AutoActionList.add(new TurnInPlaceAction(58));

            AutoActionList.add(new StoreDrivePositionAction());
            AutoActionList.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getReversedTrajectory(TrajectoryGeneratorHelper.getStraightTrajectory(.35))));
            AutoActionList.add(new AutoTurnAction(1, 3.5)); // Aim for Ball 3 with wider deadband
            AutoActionList.add(new DriveUntilPickupAction(-.2)); //previously was -.185
            AutoActionList.add(new WaitAction(.08));

            // got the ball, now go back to score it
            List<Action> Ball3Score = new ArrayList<>();
            Ball3Score.add(new ArmRotateAction(Arm.ArmTarget.SCORE_FRONT.degrees));
            //Ball3Score.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_New_T35_B5())); //old style
            //Ball3Score.add(new DriveGeneratedAction(false));

            Ball3Score.add(new DriveTrajectoryAction(TrajectoryLibrary.getInstance().get_New_T35_B5_mod2()));

            AutoActionList.add(new ParallelAction(Ball3Score));
            AutoActionList.add(new TurnInPlaceAction(-47));
            AutoActionList.add(new DriveTrajectoryAction(TrajectoryGeneratorHelper.getStraightTrajectory(.8)));
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