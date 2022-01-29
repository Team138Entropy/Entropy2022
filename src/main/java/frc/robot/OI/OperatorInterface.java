package frc.robot.OI;

import frc.robot.Constants;
import frc.robot.OI.NykoController.Axis;
import frc.robot.OI.NykoController.DPad;
import frc.robot.OI.XboxController.Button;
import frc.robot.OI.XboxController.Side;
import frc.robot.Robot;
import frc.robot.Constants.Controllers.Operator;
import frc.robot.util.LatchedBoolean;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class OperatorInterface {
    private static OperatorInterface mInstance;

    LatchedBoolean lb1 = new LatchedBoolean();
    LatchedBoolean lb2 = new LatchedBoolean();
    LatchedBoolean lb3 = new LatchedBoolean();
    LatchedBoolean lb4 = new LatchedBoolean();
    private LatchedBoolean isRightBumperPressed = new LatchedBoolean();
    private LatchedBoolean isRightTriggerPressed = new LatchedBoolean();

    // Instances of the Driver and Operator Controller
    private XboxController DriverController;
    private XboxController NewOperatorController;
    private NykoController OperatorController;
    private JoystickController joysticks;

    // Latched Booleans
    private LatchedBoolean mOperatorSelectButtonLatchedBootlean = new LatchedBoolean();

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
    public boolean getIntakeActive(){
        return NewOperatorController.getButton(Button.RB);
    }
    public double getArmPos(){
        if (NewOperatorController.getButton(Button.A)){
            return -60;
        }
        if (NewOperatorController.getButton(Button.B)){
            return 0;
        }
        if (NewOperatorController.getButton(Button.X)){
            return 60;
        }
        if (NewOperatorController.getButton(Button.Y)){
            return 110;
        }
        else {
            return 0.0;
        }
    }

    /**
     * Switches the Robot Mode
     * On the Start Button of the Operator Controller
     * @return
     */
    public boolean getSwitchModePress(){
        return mOperatorSelectButtonLatchedBootlean.update(NewOperatorController.getButton(Button.START));
    }

    public boolean getArmExtend() {
        return isRightBumperPressed.update(OperatorController.getButton(NykoController.Button.RIGHT_BUMPER));
    }

    public boolean getArmRetract() {
        return isRightTriggerPressed.update(OperatorController.getButton(NykoController.Button.RIGHT_TRIGGER));
    }

    public boolean getArmExtendManual() {
        return (OperatorController.getDPad() == NykoController.DPad.UP || OperatorController.getDPad() 
            == NykoController.DPad.UP_RIGHT) || OperatorController.getDPad() == NykoController.DPad.UP_LEFT;
    }

    public boolean getArmRetractManual() {
        return (OperatorController.getDPad() == NykoController.DPad.DOWN || OperatorController.getDPad() 
            == NykoController.DPad.DOWN_RIGHT) || OperatorController.getDPad() == NykoController.DPad.DOWN_LEFT;
    }

    public boolean isDPadOther() {
        return OperatorController.getDPad() == NykoController.DPad.OTHER;
    }
}
