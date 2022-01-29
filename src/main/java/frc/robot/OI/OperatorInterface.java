package frc.robot.OI;

import frc.robot.Constants;
import frc.robot.OI.NykoController.Axis;
import frc.robot.OI.NykoController.DPad;
import frc.robot.OI.XboxController.Button;
import frc.robot.OI.XboxController.Side;
import frc.robot.subsystems.Arm.ArmTarget;
import frc.robot.Robot;
import frc.robot.Constants.Controllers.Operator;
import frc.robot.util.LatchedBoolean;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class OperatorInterface {
    private static OperatorInterface mInstance;

    // Instances of the Driver and Operator Controller
    private XboxController DriverController;
    private XboxController NewOperatorController;
    private NykoController OperatorController;
    private JoystickController joysticks;

    // Latched Booleans
    private LatchedBoolean mOperatorSelectButton = new LatchedBoolean();
    private LatchedBoolean mLeftBumper = new LatchedBoolean();

    public static synchronized OperatorInterface getInstance() {
        if (mInstance == null) {
            mInstance = new OperatorInterface();
        }
        return mInstance;
    }
    
    private OperatorInterface() {
        DriverController = new XboxController(Constants.Controllers.Driver.port);
        NewOperatorController = new XboxController(Constants.Controllers.Operator.port);
        OperatorController = new NykoController(Constants.Controllers.Operator.port);
        //joysticks = JoystickController.getInstance();
    }

    public double getDriveThrottle() {
        return DriverController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
    }

    public double getArmX() {
        return DriverController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.X);
    }
    
    public double getDriveTurn() {
        return DriverController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
    }

    public boolean getDriveAutoSteer(){
        return DriverController.getTrigger(XboxController.Side.RIGHT);
    }

    public void setRumble(boolean a){ 
        NewOperatorController.setRumble(a);
    }

    /**
     * Returns a target arm position based on operator input. Returns null if there is no input.
     * @return
     */
    public ArmTarget getArmPos(){
        if (NewOperatorController.getButton(Button.RB)) {
            return ArmTarget.SCORE_FRONT;
        } else if (NewOperatorController.getButton(Button.LB)) {
            return ArmTarget.SCORE_BACK;
        } else if (NewOperatorController.getTrigger(Side.RIGHT)) {
            return ArmTarget.INTAKE;    
        } else {
            return null;
        }
    }

    public boolean getArmEject() {
        return NewOperatorController.getTrigger(Side.LEFT);
    }

    /**
     * Switches the Robot Mode
     * On the Start Button of the Operator Controller
     * @return
     */
    public boolean getSwitchModePress(){
        return mOperatorSelectButton.update(NewOperatorController.getButton(Button.START));
    }

    public boolean getArmExtendTest() {
        return OperatorController.getButton(NykoController.Button.RIGHT_BUMPER);
    }

    public boolean getArmRetractTest() {
        return OperatorController.getButton(NykoController.Button.RIGHT_TRIGGER);
    }

    public boolean getArmExtendManual() {
        return (OperatorController.getDPad() == NykoController.DPad.UP || OperatorController.getDPad() 
            == NykoController.DPad.UP_RIGHT) || OperatorController.getDPad() == NykoController.DPad.UP_LEFT;
    }

    public boolean getArmRetractManual() {
        return (OperatorController.getDPad() == NykoController.DPad.DOWN || OperatorController.getDPad() 
            == NykoController.DPad.DOWN_RIGHT) || OperatorController.getDPad() == NykoController.DPad.DOWN_LEFT;
    }

    public boolean getArmIntakeTest() {
        return mLeftBumper.update(OperatorController.getButton(NykoController.Button.LEFT_BUMPER));
    }

    public boolean getArmEjectTest() {
        return OperatorController.getButton(NykoController.Button.LEFT_TRIGGER);
    }
}
