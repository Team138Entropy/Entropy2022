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

public class TestDriveMode extends AutoModeBase {
    DriveTrajectoryAction dta;

    public TestDriveMode(){
        dta = new DriveTrajectoryAction(TrajectoryLibrary.getInstance().getBasicTrajectory());
    }
    
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(dta);
    }
}