package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

public class Shooter extends Subsystem {
    private static Shooter mInstance;
    WPI_TalonSRX motor = new WPI_TalonSRX(10);
    WPI_TalonSRX motor2 = new WPI_TalonSRX(9);




    public static synchronized Shooter getInstance(){
        if (mInstance == null) {
            mInstance = new Shooter();
          }
        return mInstance;
    }

    public void setPower(double power){
        motor.set(TalonSRXControlMode.PercentOutput, power);
    }

    
    private Shooter(){
    motor2.follow(motor);
    }

    public void zeroSensors() {

    }

    public void checkSubsystem() {

    }

    public void updateSmartDashBoard() {

    }
}
