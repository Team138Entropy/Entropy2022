package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
  private Talon mExtender;
  private Encoder mExtenderEncoder;

  public Arm() {
    mShoulder = new TalonSRX(0);
    mExtender = new Talon(Constants.Arm.extenderChannel);
    mExtenderEncoder = new Encoder(0, 1);
  }

  public static Arm getInstance() {
    if (mInstance == null) {
      mInstance = new Arm();
    }
    return mInstance;
  }

  public void rotateArm(double speed) {
    speed = Math.max(Math.min(speed, 1), -1);
    mShoulder.set(ControlMode.MotionMagic, speed);
  }

  public void rotateArmUp() {
    rotateArm(Constants.Arm.shoulderJogSpeed);
  }

  public void rotateArmDown() {
    rotateArm(-Constants.Arm.shoulderJogSpeed);
  }

  public void stopArmRotate() {
    rotateArm(0);
  }
  
  public void rotateArmDistance(double degrees) {
  
  }

  public void extendArm(double speed) {
    speed = Math.max(Math.min(speed, 1), -1);
    mExtender.set(speed);
  }

  public void jogArmUp() {
    extendArm(Constants.Arm.extenderJogSpeed);
  }

  public void jogArmDown() {
    extendArm(-Constants.Arm.extenderJogSpeed);
  }

  public void stopArmExtend() {
    extendArm(0);
  }
  public void extendArmDistance(double cm) {
    
  }

  public int getShoulderTicks() {
    return mShoulder.getSelectedSensorPosition();
  }

  public int getExtenderTicks() {
    return mExtenderEncoder.get();
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
