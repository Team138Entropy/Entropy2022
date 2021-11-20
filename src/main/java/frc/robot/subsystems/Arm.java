package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Encoder;
import com.playingwithfusion.TimeOfFlight;
import frc.robot.Constants;
import java.util.LinkedHashMap;
import java.util.Vector;
import frc.robot.OI.OperatorInterface;
import frc.robot.util.LatchedBoolean;
import edu.wpi.first.wpilibj.Jaguar;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Arm {
    Jaguar J0 = new Jaguar(0);
    Jaguar J1 = new Jaguar(1);
    Jaguar J2 = new Jaguar(2);
    Jaguar J3 = new Jaguar(3);
    Jaguar J4 = new Jaguar(4);
    Jaguar J5 = new Jaguar(5);
    armTalon = new TalonFX();
    shoulderTalon = new TalonFX();
    private final OperatorInterface mOperatorInterface = OperatorInterface.getInstance();
    private static Arm mInstance;
    private TimeOfFlight mLidar = new TimeOfFlight(Constants.Talons.Arm.lidarCanID);
    Encoder revEncoder = new Encoder(0, 1);
    int encoderHome = 0;
    private LatchedBoolean armUp = new LatchedBoolean();
    private LatchedBoolean armDown = new LatchedBoolean();
    private boolean inArmUpMode = false;
    private boolean inArmDownMode = false;
    private int currentEncoderPos = 0;
    //temp?
    Vector<Integer> encoderValuesQueue = new Vector<Integer>();
    public static synchronized Arm getInstance() {
      if (mInstance == null) {
      mInstance = new Arm();
      }
     return mInstance;
    }
    public Arm(){}

    private double getLidarRange(){
        return mLidar.getRange();

    }
    private int getEncoder(){
        return revEncoder.get();
    }
    private void setJaguars(double v){
        J0.set(v);
        J1.set(v);
        J2.set(v);
        J3.set(v);
        J4.set(v);
        J5.set(v);
    }
    private int encoderUpGoal(){
       return currentEncoderPos + 50;
    }
    private int encoderDownGoal(){
       return currentEncoderPos - 50;
    }
    public void periodic(){
     boolean sendArmUp = armUp.update(mOperatorInterface.isGoButtonPressed());
     boolean sendArmDown = armDown.update(mOperatorInterface.isBackButtonPressed());
        if (sendArmUp == true) { 
            inArmUpMode = true;
            currentEncoderPos = getEncoder();
        }
        if (sendArmDown == true) {
            inArmDownMode = true;
            currentEncoderPos = getEncoder();
        } 

        if (inArmUpMode) {
            System.out.println(currentEncoderPos);
            System.out.println(encoderUpGoal());
            if (getEncoder() < encoderUpGoal()) {setJaguars(0.15);} else {inArmUpMode = false;}
        }
        else if (inArmDownMode) {
            if (getEncoder() > encoderDownGoal()) {setJaguars(-0.15);} else {inArmDownMode = false;}
        }
        else {
            setJaguars(0);
        }
    }

    void testPeriodic(){
        // get the value of the joysticks Y Axis
        double yaxisValue = mOperatorInterface.shoulderExtension_value();
        shoulderTalon.set(ControlMode.PercentOutput, yaxisValue);
        double armOut = mOperatorInterface.armExtension_value();
        armTalon.set(ControlMode.PercentOutput, armOut);

    }

}