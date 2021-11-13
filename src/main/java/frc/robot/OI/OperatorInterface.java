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

    // Instances of the Driver and Operator Controller
    private final XboxController DriverController;
    private final NykoController OperatorController;

    public static synchronized OperatorInterface getInstance() {
        if (mInstance == null) {
            mInstance = new OperatorInterface();
        }
        return mInstance;
    }
    
    private OperatorInterface() {
        DriverController = new XboxController(Constants.Controllers.Driver.port);
        OperatorController = new NykoController(Constants.Controllers.Operator.port);
    }

    public double getDriveThrottle() {
        return DriverController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
      }
    
    public double getDriveTurn() {
        return DriverController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
    }

}
