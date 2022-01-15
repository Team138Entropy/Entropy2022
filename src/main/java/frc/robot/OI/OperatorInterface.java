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

    // Instances of the Driver and Operator Controller
    private XboxController DriverController;
    private XboxController NewOperatorController;
    private NykoController OperatorController;
    private JoystickController joysticks;
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
    
    public double getDriveTurn() {
        return DriverController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
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
            return null; // this errors, who did this? cant return null in the double method :pepega:
        }
    }
}
