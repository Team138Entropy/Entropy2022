package frc.robot.auto.actions;

import frc.robot.subsystems.Drive;

/**
 * Stores the Initial Drive Position Action
 * Useful for generating a backtracked position
 */
public class DriveGeneratedAction implements Action {
  private final Drive mDrive  = Drive.getInstance();
  private boolean mIsComplete;

  public DriveGeneratedAction() {
  }

  @Override
  public void start() {

  }

  @Override
  public void update() {

  }

  @Override
  public boolean isFinished() {
    return mIsComplete;
  }

  @Override
  public void done() {}
}