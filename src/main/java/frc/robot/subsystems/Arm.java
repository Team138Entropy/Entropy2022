package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;

public class Arm {
    private static Arm mInstance;

    private TalonSRX mRotator;
    private WPI_TalonSRX mExtender;

    public Arm() {
        mRotator = new TalonSRX(0);
        mExtender = new WPI_TalonSRX(Constants.Arm.extenderChannel);
        mExtender.setName("", Constants.Arm.extenderChannel);
    }

    public static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm();
        }
        return mInstance;
    }

    public void rotateArmSpeed(double speed) {
        speed = Math.max(Math.min(speed, 1), -1);
        mRotator.set(TalonSRXControlMode.Position, speed);
    }

    public void rotateArm() {
        rotateArmSpeed(Constants.Arm.rotatorSpeedDefault);
    }
    
    public void rotateArmDistance(double degrees) {
    
    }

    public void extendArmSpeed(double speed) {
        speed = Math.max(Math.min(speed, 1), -1);
        mExtender.set(speed);
    }

    public void extendArm() {
        extendArmSpeed(Constants.Arm.extenderSpeedDefault);
    }

    public void extendArmDistance(double cm) {
        
    }
}
