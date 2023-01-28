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
    private LatchedBoolean mOperatorStartButton = new LatchedBoolean();
    private LatchedBoolean mLeftBumper = new LatchedBoolean();
    private LatchedBoolean mArmRotateUp = new LatchedBoolean();
    private LatchedBoolean mArmRotateDown = new LatchedBoolean();
    private LatchedBoolean mClimberTestPress = new LatchedBoolean();
    private LatchedBoolean mOperatorClimbApprovePress = new LatchedBoolean();

    private LatchedBoolean mExtensionUp = new LatchedBoolean();
    private LatchedBoolean mExtensionDown = new LatchedBoolean();
    private LatchedBoolean mExtensionSwitchMode = new LatchedBoolean();

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

    public boolean getClimberTest(){
        return mClimberTestPress.update(mDriverController.getButton(Button.B));
    }

    public boolean getClimberTest2(){
        return mClimberTestPress.update(mDriverController.getButton(Button.Y));
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

    public boolean getDrivePrecisionSteer(){
        return mDriverController.getTrigger(Side.LEFT);
    }

    public void setOperatorRumble(boolean a){ 
        mOperatorController.setRumble(a);
    }

    public void setDriverRumble(boolean a){ 
        mDriverController.setRumble(a);
    }

    /**
     * Returns a target arm position based on operator input. Returns null if there is no input.
     * @return
     */
    public ArmTarget getArmPos(){
        if (mOperatorController.getButton(Button.LB)) {
            return ArmTarget.SCORE_FRONT;
        } else if (mOperatorController.getButton(Button.RB)) {
            return ArmTarget.SCORE_BACK;
        } else if (mOperatorController.getTrigger(Side.RIGHT)) {
            return ArmTarget.INTAKE;    
        } else if (mOperatorController.getButton(Button.Y)) {
            return ArmTarget.HOME;
        } else if (mOperatorController.getButton(Button.X)) {
            return ArmTarget.FLAT_FRONT;
        } else if (mOperatorController.getButton(Button.B)) {
            return ArmTarget.FLAT_BACK;
        }
        return null; // Return null otherwise
    }

    public boolean getArmEject() {
        return mOperatorController.getTrigger(Side.LEFT);
    }

    public boolean getGrasperIntakeManual() {
        return mLeftBumper.update(mOperatorController.getTrigger(Side.RIGHT));
    }

    public boolean intakeTeleop() {
        return mOperatorController.getTrigger(Side.RIGHT);
    }

    /**
     * Switches the Robot Mode
     * On the Start Button of the Operator Controller
     * @return
     */
    public boolean getSwitchModePress(){
        return mOperatorStartButton.update(mOperatorController.getButton(Button.START));
    }

    public boolean getSelectButtonPress(){
        return mOperatorSelectButton.update(mOperatorController.getButton(Button.BACK));
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
        return mOperatorController.getButton(Button.B);
    }

    public boolean getArmRetractManual() {
        return mOperatorController.getButton(Button.X);
    }

    public boolean getArmExtendPress(){
        return mExtensionUp.update(mOperatorController.getButton(Button.B));
    }

    public boolean getArmRetractPress(){
        return mExtensionDown.update(mOperatorController.getButton(Button.X));
    }

    public boolean getArmJogUp() {
        return mOperatorController.getButton(Button.Y);
    }

    public boolean getArmJogDown() {
        return mOperatorController.getButton(Button.A);
    }

    public boolean getArmRotateUp() {
        return mArmRotateUp.update(mDriverController.getButton(Button.Y));
    }

    public boolean getArmRotateDown() {
        return mArmRotateDown.update(mDriverController.getButton(Button.A));
    }

    public double getShoulderTargetX() {
        return mOperatorController.getJoystick(Side.RIGHT, Axis.X);
    }

    public double getShoulderTargetY() {
        return mOperatorController.getJoystick(Side.RIGHT, Axis.X);
    }

    public boolean getClimberTestExtend() {
        return mOperatorController.getButton(Button.L_JOYSTICK);
    }

    public boolean getClimberTestRetract() {
        return mOperatorController.getButton(Button.R_JOYSTICK);
    }

    public boolean getTestZeroPress(){
        return mOperatorController.getButton(Button.L_JOYSTICK);
    }

    public boolean getSwitchExtensionMode(){
        return mExtensionSwitchMode.update(mOperatorController.getButton(Button.R_JOYSTICK));
    }

    // Operator Presses to Approve Next Climbing Stage
    public boolean getOperatorClimbStageApprovePress() {
        return mOperatorClimbApprovePress.update(mOperatorController.getButton(Button.A));
    }

    public boolean getClimbCancel() {
        return mOperatorController.getButton(Button.BACK);
    }

    public boolean getGrasperCancelIntake() {
        return mOperatorController.getButton(Button.A);
    }

    public double getTeleopArmExtend() {
        return mOperatorController.getJoystick(Side.LEFT, Axis.Y);
    }

    public double getTeleopClimberExtend() {
        return mOperatorController.getJoystick(Side.RIGHT, Axis.Y);
    }

}
