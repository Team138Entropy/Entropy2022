package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;
import frc.robot.auto.TrajectoryLibrary;

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