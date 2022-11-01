package frc.robot;
import java.util.*;

/* RobotState
 * RobotState keeps track of the poses of various coordinate frames throughout the match.
 * Coordinate frame is simple a point and direction in space that defines an (x,y) coordinate system.
 * 
 * 
 * 
 */
public class RobotState {

    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }
        return mInstance;
    }

    public synchronized void outputToSmartdashboard() {
        
    }
}
