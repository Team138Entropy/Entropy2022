package frc.robot.OI;

import frc.robot.Constants;
import frc.robot.OI.XboxController.Axis;
import frc.robot.OI.XboxController.Button;
import frc.robot.OI.XboxController.Side;
import frc.robot.subsystems.Arm.ArmTarget;
import frc.robot.util.LatchedBoolean;

public class OperatorInterface {
    private static OperatorInterface mInstance;

    // Instances of the Driver and Operator Controller
    private XboxController mDriverController;
    private XboxController mOperatorController;

    // Latched Booleans
    private LatchedBoolean mOperatorSelectButton = new LatchedBoolean();
    private LatchedBoolean mLeftBumper = new LatchedBoolean();

    // Other variables
    private ArmTarget mCurrentArmTarget = ArmTarget.INTAKE;

    public static synchronized OperatorInterface getInstance() {
        if (mInstance == null) {
            mInstance = new OperatorInterface();
        }
        return mInstance;
    }
    
    private OperatorInterface() {
        mDriverController = new XboxController(Constants.Controllers.Driver.port);
        mOperatorController = new XboxController(Constants.Controllers.Operator.port);
    }

    public double getDriveThrottle() {
        return mDriverController.getJoystick(Side.LEFT, Axis.Y);
    }
    
    public double getDriveTurn() {
        return mDriverController.getJoystick(Side.RIGHT, Axis.X);
    }

    public boolean getDriveAutoSteer(){
        return mDriverController.getTrigger(Side.RIGHT);
    }

    public void setRumble(boolean a){ 
        mOperatorController.setRumble(a);
    }

    /**
     * Returns a target arm position based on operator input. Returns null if there is no input.
     * @return
     */
    public ArmTarget getArmPos(){
        if (mOperatorController.getButton(Button.RB)) {
            mCurrentArmTarget = ArmTarget.SCORE_FRONT;
        } else if (mOperatorController.getButton(Button.LB)) {
            mCurrentArmTarget = ArmTarget.SCORE_BACK;
        } else if (mOperatorController.getTrigger(Side.RIGHT)) {
            mCurrentArmTarget = ArmTarget.INTAKE;    
        }
        return mCurrentArmTarget; // Returns the last used arm target if no button is pressed
    }

    public boolean getArmEject() {
        return mOperatorController.getTrigger(Side.LEFT);
    }

    public boolean getGrasperIntakeManual() {
        return mLeftBumper.update(mOperatorController.getButton(Button.B));
    }

    /**
     * Switches the Robot Mode
     * On the Start Button of the Operator Controller
     * @return
     */
    public boolean getSwitchModePress(){
        return mOperatorSelectButton.update(mOperatorController.getButton(Button.START));
    }

    public boolean getArmExtend() {
        int input = mOperatorController.getDPad();
        return input <= 45 && input >= 315;
    }

    public boolean getArmRetract() {
        int input = mOperatorController.getDPad();
        return input <= 225 && input >= 135;
    }

    public boolean getArmExtendManual() {
        return mOperatorController.getJoystick(Side.LEFT, Axis.Y) > .5;
    }

    public boolean getArmRetractManual() {
        return mOperatorController.getJoystick(Side.LEFT, Axis.Y) < -.5;
    }

    public boolean getArmJogUp() {
        return mOperatorController.getButton(Button.Y);
    }

    public boolean getArmJogDown() {
        return mOperatorController.getButton(Button.A);
    }

    public double getShoulderTargetX() {
        return mOperatorController.getJoystick(Side.RIGHT, Axis.X);
    }

    public double getShoulderTargetY() {
        return mOperatorController.getJoystick(Side.RIGHT, Axis.X);
    }

    public boolean getClimberTestExtend(){
        return mOperatorController.getButton(Button.B);
    }

    public boolean getClimberTestRetract(){
        return mOperatorController.getButton(Button.A);
    }
}
