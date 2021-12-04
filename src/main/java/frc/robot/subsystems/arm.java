package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PWM;

public class Arm {
    private static Arm mInstance;

    private TalonSRX mRotator;
    private PWM mExtender;

    public Arm() {
        
    }

    public static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm();
        }
        return mInstance;
    }

    public void rotateArm(int degrees) {

    }

    public void rotateArm() {

    }

    public void extendArm(int cm) {
        mExtender.setPosition(cm * Constants);
    }

    public void extendArm() {

    }
}