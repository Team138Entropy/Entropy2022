package frc.robot.subsystems;

import java.util.concurrent.Callable;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.OI.OperatorInterface;
import frc.robot.subsystems.Arm.ArmExtensionTarget;
import frc.robot.subsystems.Arm.ArmTarget;
import frc.robot.util.StageExecutor;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**

 */
public class Climber extends Subsystem {

    /** 
   * All useful climber target positions during a match. 
   */
    public static enum ClimberTarget {
        LOW(0),
        ABOVE_BAR(33700),
        MID(25000),
        ABOVE_MID(30000),
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


    private double limpVariable = 0;

    // Climber Action Execution
    private final StageExecutor mClimberExecutor = new StageExecutor();

    // Execution Variables
    boolean mInitialLoopComplete = false;
    int mLoopCount = 2;
    int mCurrentLoop = 0;
    int restartStage = 1;

    // Test Values
    private double testClimbPosition = 0;
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
        
        mClimber.configMotionAcceleration(2800);
        mClimber.configMotionCruiseVelocity(2000, 10);
        
        mClimber.config_kF(0, 0, 10);
        mClimber.config_kP(0, .8, 10);
        mClimber.config_kI(0, 0, 10);
        mClimber.config_kD(0, 10, 10);

        // Initialize the Climber Executor
        mClimberExecutor.setVerboseMode();
        initBasicClimbSequence(); // basic climb instance, less agressive
        // initAdvancedClimbSequence();

    }

    // Load the Climber Stage Functions into Climber Executor
    private void initBasicClimbSequence(){
        // Prepare Climber - Verify Arm and Climber Position
        mClimberExecutor.registerStage("Prepare Climber", 
            new Callable<Boolean>() {
                public Boolean call(){
                    System.out.println("PREPARE CLIMBER WORK!");
                    // set arm to its starting position for climbing
                    mArm.rotateToPosition(100);
                    mArm.extend();

                    // set climber position to 0
                    setPosition(ClimberTarget.ABOVE_BAR.ticks);
                    return false;
                }
            },
            new Callable<Boolean>() {
                public Boolean call(){
                    // arm is in position and climber is in position
                    return mArm.isAtPosition(100) && isAtPosition(ClimberTarget.ABOVE_BAR.ticks);
                }
            },
            true
        );

        // Pull CLimber Arms Down - Requires Operator Blessing
        mClimberExecutor.registerStage("Pull Climber Arms Down", 
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
        
        // Move Arm onto High Bar - Requires Operator Blessing
        mClimberExecutor.registerStage("Move Arm onto High Bar", 
            new Callable<Boolean>() {
                public Boolean call(){
                    mArm.rotateToPosition(120);
                    return false;
                }
            },
            new Callable<Boolean>() {
                public Boolean call(){
                    return mArm.isAtPosition(113);
                }
            }, 
            true
        );
        
        // Get Off the Mid Bar and Onto High Bar
        mClimberExecutor.registerStage("Get off mid bar", 
            new Callable<Boolean>() {
                public Boolean call(){
                    // set climber position to climb position
                    setPosition(ClimberTarget.MID.ticks);
                    return false;
                }
            },
            new Callable<Boolean>() {
                public Boolean call(){
                    return isAtPosition(ClimberTarget.MID.ticks);
                }
            }, 
            false, 3
       );

      // Off Mid Bar - Swing to Other Side of High Bar
      mClimberExecutor.registerStage("Rotate to 67", 
        new Callable<Boolean>() {
            public Boolean call(){
                // set climber position to climb position
                mArm.rotateToPosition(56);
                return false;
            }
        },
        new Callable<Boolean>() {
            public Boolean call(){
                System.out.println("Stage 5 Tough Case: " + mArm.getRotation());
                return mArm.isAtPosition(59);
            }
        }, 
      false, 1 //this is likely too long
     );

    
    // SLightly Extend Arm and Get Above High Bar
    mClimberExecutor.registerStage("Pull Climber above High Bar and Retract Arm Slightly", 
        new Callable<Boolean>() {
            public Boolean call(){
                // set climber position to climb position
                setPosition(ClimberTarget.ABOVE_BAR.ticks);
                mArm.extendToPosition(ArmExtensionTarget.FULLY_EXTENDED.ticks - 10000);
                return false;
            }
        },
        new Callable<Boolean>() {
            public Boolean call(){
                return isAtPosition(ClimberTarget.ABOVE_BAR.ticks);
            }
        }, 
        false, 2
    );

    // Allow Arm to go limp
    mClimberExecutor.registerStage("Go Limp", 
        new Callable<Boolean>() {
            public Boolean call(){
                // set climber position to climb position
                //mArm.rotateToPosition(80);
                mArm.jogStop();
                //mArm.configArmVelocityAndAcceleration(20, 15);
                return false;
            }
        },
        new Callable<Boolean>() {
            public Boolean call(){
                //return mArm.isAtPosition(75);
                return true;
            }
        }, 
        true, 1.2
   );

   // Save Limp Value
   mClimberExecutor.registerStage("Save Limp Value", 
   new Callable<Boolean>() {
       public Boolean call(){
           // set climber position to climb position
           //mArm.rotateToPosition(80);
           limpVariable = mArm.getRotation();
           return false;
       }
   },
   new Callable<Boolean>() {
       public Boolean call(){
           return true;
       }
   }, 
   false,0
);


// Rotate Arm to Limp Position
mClimberExecutor.registerStage("Rotate Arm to Limp Position", 
new Callable<Boolean>() {
    public Boolean call(){                
        //mArm.rotateToPosition(mArm.getRotation() + 1);
        mArm.rotateToPosition(limpVariable - 2);
        return false;
    }
},
new Callable<Boolean>() {
    public Boolean call(){
        return true;
    }
}, 
false,0
);
   mClimberExecutor.registerStage("Pull Climber to Mid and Retract Arm", 
    new Callable<Boolean>() {
        public Boolean call(){
            // set climber position to climb position
            mArm.extend();
            setPosition(ClimberTarget.MID.ticks);
            mArm.rotateToPosition(limpVariable - 6);

            //mArm.extendToPosition(ArmExtensionTarget.ABOVE_HIGH_BAR.ticks);
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



   mClimberExecutor.registerStage("Retract arm", 
        new Callable<Boolean>() {
            public Boolean call(){
                // set climber position to climb position
                //mArm.rotateToPosition(80);
                mArm.extendToPosition(ArmExtensionTarget.UNDER_HIGH_BAR.ticks);

                return false;
            }
        },
        new Callable<Boolean>() {
            public Boolean call(){
                return mArm.isAtExtension(ArmExtensionTarget.UNDER_HIGH_BAR.ticks);
            }
        }, 
        true
   );
   mClimberExecutor.registerStage("Rotate to 102", 
        new Callable<Boolean>() {
            public Boolean call(){
                mArm.rotateToPosition(102);
                return false;
            }
        },
        new Callable<Boolean>() {
            public Boolean call(){
                return mArm.isAtPosition(102);
            }
        }, 
        true,0
   );
   mClimberExecutor.registerStage("Extend arm + goto 50", 
        new Callable<Boolean>() {
            public Boolean call(){
                mArm.extend();
                setPosition(ClimberTarget.LOW.ticks);
                return false;
            }
        },
        new Callable<Boolean>() {
            public Boolean call(){
                return mArm.isExtended() && isAtPosition(ClimberTarget.LOW.ticks);
            }
        }, 
        true

   );
    // Pull Climber Arms Down - Requires Operator Blessing
    mClimberExecutor.registerStage("Pull Climber Arms Down", 
    new Callable<Boolean>() {
        public Boolean call(){
            // set climber position to climb position
            mArm.rotateToPosition(117);
            return false;
        }
    },
    new Callable<Boolean>() {
        public Boolean call(){
            return mArm.isAtPosition(117);
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
    true,0
    );
    mClimberExecutor.registerStage("Extend to 25000", 
    new Callable<Boolean>() {
        public Boolean call(){
            // set climber position to climb position
            //mArm.rotateToPosition(90);
            setPosition(ClimberTarget.LOW.ticks);
            mArm.jogStop();
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

        
    }

    private void initAdvancedClimbSequence(){
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
            false
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
            false
        );

        mClimberExecutor.registerStage("Get off mid bar", 
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
      false,2
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
                return isAtPosition(ClimberTarget.MID.ticks) && mArm.isRetracted();
            }
        }, 
        false
    );

    mClimberExecutor.registerStage("Rotate to 75", 
        new Callable<Boolean>() {
            public Boolean call(){
                // set climber position to climb position
                //mArm.rotateToPosition(80);
                mArm.jogStop();
                return false;
            }
        },
        new Callable<Boolean>() {
            public Boolean call(){
                //return mArm.isAtPosition(75);
                return true;
            }
        }, 
        false,1
   );
   mClimberExecutor.registerStage("Extend arm out", 
        new Callable<Boolean>() {
            public Boolean call(){                
                mArm.extend();
                mArm.rotateToPosition(mArm.getRotation() + 1);
                return false;
            }
        },
        new Callable<Boolean>() {
            public Boolean call(){
                return mArm.isExtended();
            }
        }, 
        false,1
   );
   mClimberExecutor.registerStage("Retract arm", 
        new Callable<Boolean>() {
            public Boolean call(){
                // set climber position to climb position
                //mArm.rotateToPosition(80);
                mArm.retract();
                return false;
            }
        },
        new Callable<Boolean>() {
            public Boolean call(){
                return mArm.isRetracted();
            }
        }, 
        false
   );
   mClimberExecutor.registerStage("Rotate to 95", 
        new Callable<Boolean>() {
            public Boolean call(){
                mArm.rotateToPosition(97);
                return false;
            }
        },
        new Callable<Boolean>() {
            public Boolean call(){
                return mArm.isAtPosition(97);
            }
        }, 
        false,1
   );
   mClimberExecutor.registerStage("Extend arm + goto 50", 
        new Callable<Boolean>() {
            public Boolean call(){
                mArm.extend();
                setPosition(ClimberTarget.LOW.ticks);
                return false;
            }
        },
        new Callable<Boolean>() {
            public Boolean call(){
                return mArm.isExtended() && isAtPosition(ClimberTarget.LOW.ticks);
            }
        }, 
        false

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
    true,4
    );
    mClimberExecutor.registerStage("Extend to 25000", 
    new Callable<Boolean>() {
        public Boolean call(){
            // set climber position to climb position
            //mArm.rotateToPosition(90);
            setPosition(ClimberTarget.LOW.ticks);
            mArm.jogStop();
            return false;
        }
    },
    new Callable<Boolean>() {
        public Boolean call(){
            return isAtPosition(ClimberTarget.LOW.ticks);
        }
    }, 
    false
    );

        
    }

    public synchronized void reset(){
        mClimberExecutor.reset();
        mInitialLoopComplete = false;
    }

    // Update runs the Climber State Machine
    // Operator is able to stop the state machine 
    public synchronized void update(boolean stop, boolean accept){
        mClimberExecutor.update(accept);

        // Rumble Controller
        mOperatorInterface.setOperatorRumble(mClimberExecutor.needUserInputToStart());
        SmartDashboard.putBoolean("Climber Needs Manual Input", mClimberExecutor.needUserInputToStart());


        
        // Climb is Complete, this is the initial setup loop
        if(mClimberExecutor.isComplete() && !mInitialLoopComplete){
            System.out.println("Climbing! Initial Complete!");
            // the climbing execution wants to occur twice
            /*
 
            mCurrentLoop++;
            if(mCurrentLoop < mLoopCount){
                // go reset 
                mClimberExecutor.resetToStage(restartStage);
                mInitialLoopComplete = false;
            }else{
                mInitialLoopComplete = false;
            }

            */

            SmartDashboard.putBoolean("TestUpdate", false);
            SmartDashboard.putNumber("Test_ArmTarget", mArm.getRotation());
            SmartDashboard.putNumber("Test_ClimberTarget", this.getClimberPosition());
            SmartDashboard.putBoolean("Test_IsExtended", mArm.isExtended());
            testDegreesPosition = mArm.getRotation();
            testExtend = mArm.isExtended();
            testClimbPosition = this.getClimberPosition();
            mInitialLoopComplete = true;
            testInUpdateLoop = true;
        }
/*
        // TEST
        // check for test loop update
        if(SmartDashboard.getBoolean("TestUpdate", false)){
            System.out.println("Test Update Init!");
            // load in all test values
            testDegreesPosition = SmartDashboard.getNumber("Test_ArmTarget", 0);
            testClimbPosition = SmartDashboard.getNumber("Test_ClimberTarget", 0);
            testExtend = SmartDashboard.getBoolean("Test_IsExtended", false);

            // turn off toggle
            SmartDashboard.putBoolean("TestUpdate", false);
        }

        // test manual input - values progated over from test
        if(testInUpdateLoop){
            mArm.rotateToPosition(testDegreesPosition);
            setPosition((int) testClimbPosition);
            if(testExtend){
                mArm.extend();
            }else{
                mArm.retract();
            }
        }
        SmartDashboard.putBoolean("Climber Test Loop", testInUpdateLoop);
        */
    }


    // Set Position into logic
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
        //return getClimberPosition() >= encoderPosition - 500 && getClimberPosition() <= encoderPosition + 500;
        return true;
    }

    public void updateSmartDashBoard() {
        SmartDashboard.putNumber("Climber Stage", mClimberExecutor.getCurrentStage()); // Climber Stage Number
        SmartDashboard.putString("Climber Stage Name", mClimberExecutor.getCurrentStageName()); // Climber Stage Name
        SmartDashboard.putBoolean("Climb Complete", mClimberExecutor.isComplete()); // Climb executor is complete
        SmartDashboard.putNumber("Climber Position", getClimberPosition()); // Climber Encoder Position
        SmartDashboard.putNumber("Climber Velocity", mClimber.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Climber Winch Output Percentage", mClimber.getMotorOutputPercent());
        SmartDashboard.putNumber("Climber Winch Voltage Percentage", mClimber.getMotorOutputVoltage());
        SmartDashboard.putNumber("Climber Closed Loop Error", mClimber.getClosedLoopError());
        SmartDashboard.putNumber("Climber Winch Current", mClimber.getSupplyCurrent());
    }
}
