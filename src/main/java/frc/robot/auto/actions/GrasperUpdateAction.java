package frc.robot.auto.actions;
import frc.robot.Constants;
import frc.robot.subsystems.Grasper;
import edu.wpi.first.wpilibj.Timer;

/**
 * Continues to allow the grasper to update
 */
public class GrasperUpdateAction implements Action {
  private final Grasper mGrasper  = Grasper.getInstance();
  private double mSecondsDuration;
  private Timer mTimer;

  public GrasperUpdateAction(double durationSeconds) {
    mSecondsDuration = durationSeconds;
  }

  @Override
  public void start() {
    mTimer = new Timer();
    mTimer.start();
  }

  @Override
  public void update() {
    mGrasper.update(Constants.Grasper.globelPowerDistribution.getCurrent(Constants.Grasper.powerDistributionNumber));
    System.out.println("grasper updating");
  }

  @Override
  public boolean isFinished() {
    return mTimer.hasElapsed(mSecondsDuration);
  }

  @Override
  public void done() {}
}
