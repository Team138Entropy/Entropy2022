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
    int driveControllerInUse = 0;
    int operatorControllerInUse = 0;
    // Instances of the Driver and Operator Controller
    private XboxController xbox1;
    private XboxController xbox2;
    private NykoController Nyko1;
    private JoystickController joysticks;
    private Wheel wheel;
    private uselessController controllerThing;
    public static synchronized OperatorInterface getInstance() {
        if (mInstance == null) {
            mInstance = new OperatorInterface();
        }
        return mInstance;
    }
    
    private OperatorInterface() {
        xbox2 = new XboxController(Constants.Controllers.Operator.port);
        xbox1 = new XboxController(Constants.Controllers.Driver.port);
        //Nyko1 = new NykoController(Constants.Controllers.Operator.port);
        joysticks = JoystickController.getInstance();
        wheel = Wheel.getInstance();
        controllerThing = uselessController.getInstance();
    }
    public void setDriveControllerInUse(int value) { 
        driveControllerInUse = value;
    }
    public int getDriveControllerInUse() {
        return driveControllerInUse;
    }
    public void setOperatorControllerInUse(int value) { 
        operatorControllerInUse = value;
    }
    public int getOperatorControllerInUse() {
        return operatorControllerInUse;
    }
    public double getDriveThrottle() {
        if (driveControllerInUse == 0){
            return xbox1.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
        } 
        if (driveControllerInUse == 1){
            return joysticks.getYJoyStick_L();
        }
        if (driveControllerInUse == 2){
            return wheel.getYWheel();
        }
        if (driveControllerInUse == 3){
            return controllerThing.getY_thing();
        }
        return 0;
      }
    
    public double getDriveTurn() {
        if (driveControllerInUse == 0) {  
             return xbox1.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
        }
        if (driveControllerInUse == 1) {
             return joysticks.getXJoyStick_R();
        }
        if (driveControllerInUse == 2){
            return wheel.getXWheel();
        }
        if (driveControllerInUse == 3){
            return controllerThing.getX_thing();
        }
        return 0;
    }
    //Add functions for the operator controller




}
