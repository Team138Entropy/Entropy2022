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
        LOW(50),
        ABOVE_BAR(33700),
        MID(25000),
        MID2(15000)
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

    // Test Values
    private int testClimbPosition = 0;
    private double testDegreesPosition = 0;
    private boolean testExtend = false;
    private boolean testInUpdateLoop = false;

    public static synchronized Climber getInstance(){
        if (mInstance == null) {
            mInstance = new Climber();
          }
        return mInstance;
    }
    
    private Climber(){
        mClimber = new TalonSRX(Constants.Talons.Climber.climber);
        mClimber.configFactoryDefault();
        mClimber.setNeutralMode(NeutralMode.Brake);

        // Make a Rotate to Position which is given an encoder position
        
        mClimber.configMotionAcceleration(1200);
        mClimber.configMotionCruiseVelocity(1200, 10);
        
        mClimber.config_kF(0, 0, 10);
        mClimber.config_kP(0, .8, 10);
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
                    mArm.rotateToPosition(95);
                    if(mArm.isAtPosition(95)){
                        mArm.extend();
                    }
                    // set climber position to 0
                    setPosition(ClimberTarget.ABOVE_BAR.ticks);
                    return false;
                }
            },
            new Callable<Boolean>() {
                public Boolean call(){
                    System.out.println("IS DONE FUNCTION!?");
                    // arm is in position and climber is in position
                    System.out.println(mArm.isAtPosition(95));
                    System.out.println(isAtPosition(ClimberTarget.ABOVE_BAR.ticks));
                    return mArm.isAtPosition(95) && isAtPosition(ClimberTarget.ABOVE_BAR.ticks);
                }
            },
            true
        );

        // Lift Climber Arms - Requires Operator Blessing
        mClimberExecutor.registerStage("Lift Climber Arms", 
            new Callable<Boolean>() {
                public Boolean call(){
                    // set climber position to climb position
                    setPosition(ClimberTarget.LOW.ticks);
                    return false;
                }
            },
            new Callable<Boolean>() {
                public Boolean call(){
                    return isAtPosition(ClimberTarget.LOW.ticks);
                }
            }, 
            true
        );
        
        // Pull Climber Arms Down - Requires Operator Blessing
        mClimberExecutor.registerStage("Pull Climber Arms Down", 
            new Callable<Boolean>() {
                public Boolean call(){
                    // set climber position to climb position
                    mArm.rotateToPosition(115);
                    return false;
                }
            },
            new Callable<Boolean>() {
                public Boolean call(){
                    return mArm.isAtPosition(115);
                }
            }, 
            true
        );

        mClimberExecutor.registerStage("Extend to 25000", 
            new Callable<Boolean>() {
                public Boolean call(){
                    // set climber position to climb position
                    //mArm.rotateToPosition(90);
                    setPosition(ClimberTarget.MID.ticks);
                    return false;
                }
            },
            new Callable<Boolean>() {
                public Boolean call(){
                    return isAtPosition(ClimberTarget.MID.ticks);
                }
            }, 
            true
       );

       mClimberExecutor.registerStage("Extend to 15000", 
        new Callable<Boolean>() {
            public Boolean call(){
                // set climber position to climb position
                setPosition(ClimberTarget.MID2.ticks);
                return false;
            }
        },
        new Callable<Boolean>() {
            public Boolean call(){
                return isAtPosition(ClimberTarget.MID2.ticks);
            }
        }, 
        true
      );

      mClimberExecutor.registerStage("Rotate to 45", 
        new Callable<Boolean>() {
            public Boolean call(){
                // set climber position to climb position
                mArm.rotateToPosition(45);
                return false;
            }
        },
        new Callable<Boolean>() {
            public Boolean call(){
                return mArm.isAtPosition(45);
            }
        }, 
      true
     );

     mClimberExecutor.registerStage("Pull Climber to Mid and Retract Arm", 
        new Callable<Boolean>() {
            public Boolean call(){
                // set climber position to climb position
                setPosition(ClimberTarget.MID.ticks);
                mArm.retract();
                return false;
            }
        },
        new Callable<Boolean>() {
            public Boolean call(){
                return isAtPosition(ClimberTarget.MID.ticks);
            }
        }, 
        true
    );

    mClimberExecutor.registerStage("Rotate to 75", 
        new Callable<Boolean>() {
            public Boolean call(){
                // set climber position to climb position
                mArm.rotateToPosition(75);
                return false;
            }
        },
        new Callable<Boolean>() {
            public Boolean call(){
                return mArm.isAtPosition(75);
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

        // test manual input - values progated over from test
        if(testInUpdateLoop){
            mArm.rotateToPosition(testDegreesPosition);
            setPosition(testClimbPosition);
            if(testExtend){
                mArm.extend();
            }else{
                mArm.retract();
            }
        }
        SmartDashboard.putBoolean("Climber Test Loop", testInUpdateLoop);
    }

    public synchronized void setPosition(int pos){
        mClimber.set(ControlMode.MotionMagic, pos, DemandType.ArbitraryFeedForward, 0.1);
        SmartDashboard.putNumber("Climber Target", pos);
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
        return getClimberPosition() >= encoderPosition + 1000 || getClimberPosition() <= encoderPosition + 1000;
    }

    public void updateSmartDashBoard() {
        SmartDashboard.putNumber("Climber Stage", mClimberExecutor.getCurrentStage()); // Climber Stage Number
        SmartDashboard.putString("Climber Stage Name", mClimberExecutor.getCurrentStageName()); // Climber Stage Name
        SmartDashboard.putBoolean("Climb Complete", mClimberExecutor.isComplete()); // Climb executor is complete
        SmartDashboard.putNumber("Climber Position", getClimberPosition()); // Climber Encoder Position
        SmartDashboard.putNumber("Climber Velocity", mClimber.getSelectedSensorVelocity());
    }
}
