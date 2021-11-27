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

public class TestDriveMode extends AutoModeBase {
    DriveTrajectoryAction dta;

    public TestDriveMode(){
        // Create new trajectory 
        //dta = new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getBasicTrajectory());
        
        double metersPerSecond = 1.8288;
        double acceleratoionMetersPerSecondSquared = .5;
        
        TrajectoryConfig config =
        new TrajectoryConfig(metersPerSecond,
        acceleratoionMetersPerSecondSquared);


        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(1, 0, new Rotation2d(0))
            ),
            // End 3 meters straight ahead of where we started, facing forward
            // Pass config
            config
        );

        dta = new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getBasicTrajectory());
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(dta);
    }
}