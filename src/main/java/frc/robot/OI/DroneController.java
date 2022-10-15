package frc.robot.OI;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;


public class DroneController {
    private final Joystick mController;


    public enum Side {
        LEFT,
        RIGHT
      }
    
      public enum Axis {
        X,
        Y
      }
    
      public enum DPad {
        UP,
        DOWN,
        LEFT,
        RIGHT,
        UP_LEFT,
        UP_RIGHT,
        DOWN_LEFT,
        DOWN_RIGHT,
        OTHER
      }
    
      public enum Button {
        BUTTON_1(1),
        BUTTON_2(2),
        BUTTON_3(3),
        BUTTON_4(4),
        LEFT_BUMPER(5),
        RIGHT_BUMPER(6),
        LEFT_TRIGGER(7),
        RIGHT_TRIGGER(8),
        MIDDLE_9(9),
        MIDDLE_10(10),
        MIDDLE_11(11),
        LEFT_JOYSTICK(12),
        RIGHT_JOYSTICK(13);
    
        public final int id;
    
        Button(int id) {
          this.id = id;
        }
      }
    
      // Pass in the port of the Controller
      public DroneController(int portArg) {
        mController = new Joystick(portArg);
        checkNameAndPort();
      }
    
      private boolean alreadyWarnedInSimulator = false;
    
      public boolean checkNameAndPort() {
        // for some stupid reason, the 300iq people at nyko thought it would be cool and epic to put a
        // tab character after the name of the controller that is reported. the driver station then
        // passes this on to us, so we get something that makes no sense and is bad to debug. is it one
        // space? ten? a tab? something else stupid? we trim the string away
        /*
        String name = mController.getName().trim();
        if (!name.equals(Constants.Controllers.Operator.name) || mController.getPort() != 1) {
          if (RobotBase.isReal()) {
            DriverStation.reportError(
                "Airflo Controller not found in port 1! Got name "
                    + name
                    + " in port "
                    + mController.getPort(),
                new Error().getStackTrace());
          } else {
            if (!alreadyWarnedInSimulator)
              DriverStation.reportWarning(
                  "Airflo Controller not found in port 1! Got name "
                      + name
                      + " in port "
                      + mController.getPort()
                      + " (not reporting error due to simulated environment)",
                  new Error().getStackTrace());
            alreadyWarnedInSimulator = true;
          }
          return false;
        }
        */
        return true;
      }

      /*
       * RIGHT:
       *    Raw Axis 0, Raw Axis 1 (X == RAW 0, Y == RAW 1)
       * 
       * LEFT:
       *    Raw Axis 3, Raw Axis 2 (X == RAW 3, Y == RAW 2)
       */
    
      double getJoystick(Side side, Axis axis) {
        double deadband = Constants.Controllers.joystickDeadband;

        int rawAxisID = 0;
        if(Side.LEFT == side)
        { //check if Axis is X, if not go to the right axis
            rawAxisID = (Axis.X == axis) ? 3 : 2;
        }else if(Side.RIGHT == side)
        {
            rawAxisID = (Axis.X == axis) ? 0 : 1;
        }
    
        // multiplies by -1 if y-axis (inverted normally)
        return handleDeadband(
           mController.getRawAxis(rawAxisID), deadband);
      }
    
      boolean getButton(Button button) {
        return mController.getRawButton(button.id);
      }
    
     
    
      private double handleDeadband(double value, double deadband) {
        return (Math.abs(value) > Math.abs(deadband)) ? value : 0;
      }
    }
    

