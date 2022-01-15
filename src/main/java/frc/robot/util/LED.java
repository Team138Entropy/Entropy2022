package frc.robot.util;

import edu.wpi.first.wpilibj.PWM;
import frc.robot.Constants.Misc;

public class LED {
    private static LED mInstance;
    private static PWM mLED_ring = new PWM(Misc.LED_Channel);

    private LED(){} // empty constructor

    public static synchronized LED getInstance(){
        if (mInstance == null){
            mInstance = new LED();
        }
        return mInstance;
    }

    public double getPercentage(){
        return mLED_ring.getRaw();
    }

    public void setPercentage(int amount){
        if (amount > 255 || amount < 0){
            System.out.println("Invalid amount for setPercentage:" + amount);

        } else {
            mLED_ring.setRaw(amount);
        }
    }
}
