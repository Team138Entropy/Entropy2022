package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import frc.robot.Constants;

/**
 * Arm subsystem comprised of a rotating shoulder and telescoping extender.
 */
public class Arm extends Subsystem {
  private static Arm mInstance;

  private TalonSRX mShoulder;
  private Talon mForearm;
  private Encoder mForearmEncoder;

  public Arm() {
    mShoulder = new TalonSRX(0);
    mForearm = new Talon(Constants.Arm.forearmChannel);
    mForearmEncoder = new Encoder(0, 1);
  }

  public static Arm getInstance() {
    if (mInstance == null) {
      mInstance = new Arm();
    }
    return mInstance;
  }

  public void rotate(double speed) {
    speed = Math.max(Math.min(speed, 1), -1);
    mShoulder.set(ControlMode.PercentOutput, speed);
  }

  public void rotateDistance(double degrees) {
    degrees = Math.max(Math.min(degrees, (Constants.Arm.maxEncoderPositionShoulder - getShoulderPosition()) / Constants.Arm.ticksPerDegreeShoulder), -getShoulderPosition() / Constants.Arm.ticksPerDegreeShoulder);
    mShoulder.set(ControlMode.MotionMagic, getShoulderPosition() + degrees * Constants.Arm.ticksPerDegreeShoulder, DemandType.ArbitraryFeedForward, getGravityFeedForward());
  }

  private double getGravityFeedForward() {
    int currentPos = getShoulderPosition() - Constants.Arm.positionHorizontal;
    double currentRads = currentPos * Constants.Arm.ticksPerRadianShoulder;
    double ff = Constants.Arm.maxGravityFF * Math.cos(currentRads);
    return ff;
  }

  public void jogRotateUp() {
    rotate(Constants.Arm.jogSpeedShoulder);
  }

  public void jogRotateDown() {
    rotate(-Constants.Arm.jogSpeedShoulder);
  }

  public void stopShoulder() {
    rotate(0);
  }

  public void extend(double speed) {
    speed = Math.max(Math.min(speed, 1), -1);
    mForearm.set(speed);
  }

  public void extendDistance(double cm) {
    // mForearm.setPosition(cm );
  }

  public void jogUp() {
    extend(Constants.Arm.forearmJogSpeed);
  }

  public void jogDown() {
    extend(-Constants.Arm.forearmJogSpeed);
  }

  public void stopForearm() {
    extend(0);
  }

  public int getShoulderPosition() {
    return mShoulder.getSelectedSensorPosition();
  }

  public int getForearmPosition() {
    return mForearmEncoder.get();
  }

  @Override
  public void zeroSensors() {
    // TODO Auto-generated method stub
  }

  @Override
  public void checkSubsystem() {
    // TODO Auto-generated method stub
  }
}
