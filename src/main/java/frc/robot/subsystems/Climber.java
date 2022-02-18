package frc.robot.subsystems;

import java.util.concurrent.Callable;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.OI.OperatorInterface;
import frc.robot.util.StageExecutor;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**

 */
public class Climber extends Subsystem {

    /** 
   * All useful climber target positions during a match. 
   */
    public static enum ClimberTarget {
        LOW(200),
        ABOVE_BAR(33760)
        ;
        public int ticks;

        private ClimberTarget(int ticks) {
            this.ticks = ticks;
        }
    }
    private static Climber mInstance;

    private final TalonSRX mClimber;

    // Reference to the Operator Interface
    private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();

    // Reference to the Arm
    private final Arm mArm = Arm.getInstance();

    // Climber Action Execution
    private final StageExecutor mClimberExecutor = new StageExecutor();

    public static synchronized Climber getInstance(){
        if (mInstance == null) {
            mInstance = new Climber();
          }
        return mInstance;
    }
    
    private Climber(){
        mClimber = new TalonSRX(Constants.Talons.Climber.climber);
        mClimber.setNeutralMode(NeutralMode.Brake);

        // Make a Rotate to Position which is given an encoder position
        mClimber.configMotionAcceleration(20000);
        mClimber.configMotionCruiseVelocity(20000, 10);

        mClimber.config_kF(0, 0, 10);
        mClimber.config_kP(0, 0, 10);
        mClimber.config_kI(0, 0, 10);
        mClimber.config_kD(0, 0, 10);

        // Initialize the Climber Executor
        mClimberExecutor.setVerboseMode();
        initClimberExecutor();
    }

    // Load the Climber Stage Functions into Climber Executor
    private void initClimberExecutor(){
        // Prepare Climber - Verify Arm and Climber Position
        mClimberExecutor.registerStage("Prepare Climber", 
            new Callable<Boolean>() {
                public Boolean call(){
                    System.out.println("PREPARE CLIMBER WORK!");
                    // set arm to its starting position for climbing
                    mArm.rotateToPosition(180);

                    // set climber position to 0
                    setPosition(0);
                    return false;
                }
            },
            new Callable<Boolean>() {
                public Boolean call(){
                    System.out.println("IS DONE FUNCTION!?");
                    // arm is in position and climber is in position
                    return true;
                }
            },
            true
        );

        // Lift Climber Arms - Requires Operator Blessing
        mClimberExecutor.registerStage("Lift Climber Arms", 
            new Callable<Boolean>() {
                public Boolean call(){
                    // set climber position to climb position
                    setPosition(22000);
                    return false;
                }
            },
            new Callable<Boolean>() {
                public Boolean call(){
                    return isAtPosition(22000);
                }
            }, 
            true
        );
        
        // Pull Climber Arms Down - Requires Operator Blessing
        mClimberExecutor.registerStage("Pull Climber Arms Down", 
            new Callable<Boolean>() {
                public Boolean call(){
                    // set climber position to climb position
                    setPosition(0);
                    return false;
                }
            },
            new Callable<Boolean>() {
                public Boolean call(){
                    return isAtPosition(0);
                }
            }, 
            true
        );

        
    }

    public synchronized void reset(){
        mClimberExecutor.reset();
    }

    // Update runs the Climber State Machine
    // Operator is able to stop the state machine 
    public synchronized void update(boolean stop, boolean accept){
        mClimberExecutor.update(accept);

        // Rumble Controller
        mOperatorInterface.setOperatorRumble(mClimberExecutor.needUserInputToStart());
        SmartDashboard.putBoolean("Climber Needs Manual Input", mClimberExecutor.needUserInputToStart());
    }

    public synchronized void setPosition(int pos){
        mClimber.set(ControlMode.MotionMagic, pos, DemandType.ArbitraryFeedForward, 0.1);
    }

    // Extends the Climber Slowly
    // for use in test mode only
    public synchronized void TestExtend(){
        mClimber.set(ControlMode.PercentOutput, Constants.Climber.TestExtendOutput);
    }

    // Retracts the Climber Slowly
    // for use in test mode only
    public synchronized void TestRetract(){
        mClimber.set(ControlMode.PercentOutput, -Constants.Climber.TestRetractOutput);
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

    public boolean isAtPosition(int encoderPosition){
        return false;
    }

    public void updateSmartDashBoard() {
        SmartDashboard.putNumber("Climber Stage", mClimberExecutor.getCurrentStage()); // Climber Stage Number
        SmartDashboard.putString("Climber Stage Name", mClimberExecutor.getCurrentStageName()); // Climber Stage Name
        SmartDashboard.putBoolean("Climb Complete", mClimberExecutor.isComplete()); // Climb executor is complete
        SmartDashboard.putNumber("Climber Position", getClimberPosition()); // Climber Encoder Position
        SmartDashboard.putNumber("Climber Velocity", mClimber.getSelectedSensorVelocity());
    }
}
