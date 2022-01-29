package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Climber is intended to be mostly autonomous
 * Stage 1 - 
 *        Climber is raising to climbing position
 *        Arm is in low position
 *        Complete when arm is at full extension
 * Stage 2 -
 *        Climber is pulling up to top bar
 *        Arm is in low position
 *        Complete when climber is at top
 * Stage 3 - 
 *      Climber is holding position
 *      Arm is rotating to top bar
 *      TBD: How to determind if arm is on top bar
 *      arm might need to intake and and then extend
 */
public class Climber extends Subsystem {
    private static Climber mInstance;

    public enum ClimberStage {
        Stage1("Stage1_RaiseArms"),
        Stage2("Stage2_PullUp"),
        Stage3("Stage3_PositionArm"),
        Stage4("Stage4_PullArm"),
        Idle("Idle")
        ;

        private final String text;
    
        /**
         * @param text
         */
        ClimberStage(final String text) {
            this.text = text;
        }
    
        /* (non-Javadoc)
         * @see java.lang.Enum#toString()
         */
        @Override
        public String toString() {
            return text;
        }
    }
    private ClimberStage mCurrentStage = ClimberStage.Stage1;

    private TalonSRX mClimber;

    public static synchronized Climber getInstance(){
        if (mInstance == null) {
            mInstance = new Climber();
          }
        return mInstance;
    }
    
    private Climber(){
        mClimber = new TalonSRX(Constants.Talons.Climber.climber);
    }

    // stop button maybe?
    public synchronized void update(){
        switch(mCurrentStage){
            case Stage1:
                // Robot is below bar, extending arms to go up
                
            break;
            case Stage2: 

            break;
            case Stage3: 

            break;
            case Stage4: 

            break;
            case Idle:
                // Do Nothing
            break;
            default:
                System.out.println("Error: Unreconizied Climber Stage!");
                break;
        }
    }

    // Extends the Climber Slowly
    // for use in test mode only
    public synchronized void TestExtend(){
        mClimber.set(ControlMode.PercentOutput, Constants.Climber.TestExtendOutput);
    }

    // Retracts the Climber Slowly
    // for use in test mode only
    public synchronized void TestRetract(){
        mClimber.set(ControlMode.PercentOutput, Constants.Climber.TestRetractOutput);
    }

    // Stops the Climber
    // for use in test mode only
    public synchronized void TestStop(){
        mClimber.set(ControlMode.PercentOutput, 0);
    }

    // Called on TeleopInit
    // Expects climber to be in low position
    public void zeroSensors() {
        mClimber.setSelectedSensorPosition(0);
    }

    public void checkSubsystem() {

    }

    public double getClimberPosition(){
        return mClimber.getSelectedSensorPosition();
    }

    public void updateSmartDashBoard() {
        SmartDashboard.putString("Climber Stage", mCurrentStage.toString());
        SmartDashboard.putNumber("Climber Position", getClimberPosition());
    }
}
