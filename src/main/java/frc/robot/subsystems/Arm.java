package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Encoder;
import com.playingwithfusion.TimeOfFlight;
import frc.robot.Constants;

public class Arm {
    private TimeOfFlight mLidar = new TimeOfFlight(Constants.Talons.Arm.lidarCanID);
    Encoder revEncoder = new Encoder(0, 1);

    private static Arm sInstance;
    public static synchronized Arm getInstance() {
        if (sInstance == null) {
            sInstance = new Arm();
        }
        return sInstance;
    }

    public Arm(){}

    public double getLidarRange(){
        return mLidar.getRange();

    }

    public void periodic(){
        System.out.println("Lidar range: " + getLidarRange());
        System.out.println("Encoder thing: " + revEncoder.get());
    }

}