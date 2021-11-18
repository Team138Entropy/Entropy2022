package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Encoder;
import com.playingwithfusion.TimeOfFlight;
import frc.robot.Constants;
import java.util.LinkedHashMap;
import java.util.Vector;

public class Arm {
    private TimeOfFlight mLidar = new TimeOfFlight(Constants.Talons.Arm.lidarCanID);
    Encoder revEncoder = new Encoder(0, 1);
    int encoderHome = 0;

    //temp?
    Vector<Integer> encoderValuesQueue = new Vector<Integer>();

    public Arm(){}

    private double getLidarRange(){
        return mLidar.getRange();

    }
    private int getEncoder(){
        return revEncoder.get();
    }

    public double getEncoderValueQueueAverage(){
        int sum = 0;
        for(int i=0; i<=encoderValuesQueue.size()-1; i++) {
            sum += encoderValuesQueue.get(i);
        }

        return sum/encoderValuesQueue.size();
    }

    private void cleanEncoderValueQueue(){
        if (encoderValuesQueue.size() > 100){
            while (encoderValuesQueue.size() > 100){
                encoderValuesQueue.remove(0);
            }
        }
    }

    

    public void periodic(){
        if (getLidarRange() < 70){
            // Hit (probably)
            System.out.println("Hit, encoder: " + getEncoder());
            encoderHome = getEncoder();
            encoderValuesQueue.add(getEncoder());
            cleanEncoderValueQueue();
        }

    }

}