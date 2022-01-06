package frc.robot.OI;

import frc.robot.Constants;
import frc.robot.OI.NykoController.Axis;
import frc.robot.OI.NykoController.DPad;
import frc.robot.OI.XboxController.Button;
import frc.robot.OI.XboxController.Side;
import frc.robot.Robot;
import frc.robot.Constants.Controllers.Operator;
import frc.robot.util.LatchedBoolean;

public class OperatorInterface {
    private static OperatorInterface mInstance;

    LatchedBoolean lb1 = new LatchedBoolean();
    LatchedBoolean lb2 = new LatchedBoolean();
    LatchedBoolean lb3 = new LatchedBoolean();
    LatchedBoolean lb4 = new LatchedBoolean();

    // Instances of the Driver and Operator Controller
    private XboxController DriverController;
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
        OperatorController = new NykoController(Constants.Controllers.Operator.port);
        //joysticks = JoystickController.getInstance();
    }

    public double getDriveThrottle() {
        return DriverController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
      }
    
    public double getDriveTurn() {
        return DriverController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
    }

    /**
     * Return true if Button 1 is pressed.
     */
    public boolean getButton1() {
        return lb1.update(OperatorController.getButton(NykoController.Button.BUTTON_1));
    }

    /**
     * Return true if Button 2 is pressed.
     */
    public boolean getButton2() {
        return lb2.update(OperatorController.getButton(NykoController.Button.BUTTON_2));

    }

    /**
     * Return true if Button 3 is pressed.
     */
    public boolean getButton3() {
        return lb3.update(OperatorController.getButton(NykoController.Button.BUTTON_3));
    }

    /**
     * Return true if Button 4 is pressed.
     */
    public boolean getButton4() {
        return lb4.update(OperatorController.getButton(NykoController.Button.BUTTON_4));
    }
}
