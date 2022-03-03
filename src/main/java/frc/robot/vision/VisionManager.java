package frc.robot.vision;

import java.util.concurrent.ConcurrentHashMap;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadPoolExecutor;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.networking.UDPReciever;


import frc.robot.Constants;

public class VisionManager {

    // UDP Reciever used in vision manager

    // Target Data
    private ConcurrentHashMap<Constants.TargetType, TargetInfo> mTargetData;

    // Current Selected Target
    private Constants.TargetType mSelectedTarget = Constants.TargetType.CAMERA_1_BLUE_CARGO;

    private static VisionManager mInstance;

    public static synchronized VisionManager getInstance() {
      if (mInstance == null) {
        mInstance = new VisionManager();
      }
      return mInstance;
    }

    private UDPReciever PacketReciever;
    private JSONParser parser;

    
  // Lock that protects the parser and allows one user at a time
  private final Object ParserLock = new Object();
  
  // Exector Thread
  // Packets are processed in this thread!
  private ThreadPoolExecutor executor = (ThreadPoolExecutor) Executors.newFixedThreadPool(2);
  private ThreadPoolExecutor RobotTrackerExecutor =
      (ThreadPoolExecutor) Executors.newFixedThreadPool(2);

    private VisionManager(){
        initTargetStorage();

            // Start a Socket to listen to UDP Packet
            // Each thread pass to a processer which
            PacketReciever = new UDPReciever("127.0.0.1", 5800);
            parser = new JSONParser();

            Thread listenerThread =
                new Thread(
                    new Runnable() {
                    @Override
                    public void run() {
                        ProcessPacket();
                    }
                    });

            // Set Name and Start Thread!
            listenerThread.setName("VisionListenerThread");
            listenerThread.setPriority(Thread.MIN_PRIORITY + 2);
            listenerThread.start();
    }

    private void ParsePacket(String packet) {
        try {
          JSONObject CurrentPacket;
    
          // Appears the Parser Object isn't thread safe
          // Make sure the parse function is only called one at a time
          synchronized (ParserLock) {
            CurrentPacket = (JSONObject) parser.parse(packet);
          }
    
          // Attempt to form target info object
          // is possible this fails!
          TargetInfo ti;
          try {
            // Parse Target Info
            ti =
                new TargetInfo(
                    ((Number) CurrentPacket.get("cameraid")).intValue(),
                    ((Number) CurrentPacket.get("BallY")).doubleValue(),
                    ((Number) CurrentPacket.get("BallX")).doubleValue(),
                    ((Number) CurrentPacket.get("Dist")).doubleValue());

            // Convert Ball Color to the Associated Enum
            // right now only one camera is supported, so only camera enums are used
            Constants.TargetType targType;
            String BallColor = ((String) CurrentPacket.get("ballColor").toString()).toLowerCase();
            if(BallColor.equals("red")){
              targType = Constants.TargetType.CAMERA_1_RED_CARGO;
            }else{
              targType = Constants.TargetType.CAMERA_1_BLUE_CARGO;
            }
    
            // Reconvert Field information
            ti.CalculateFields();   
    
            // Pass to executor to not block up this field
            RobotTrackerExecutor.execute(
                new Runnable() {
                  public void run() {
                    // If we made it to this point we had all the required keys!
                    // Now we need to update RobotState with our new values!
                    // mRobotTracker.addVisionUpdate(Timer.getFPGATimestamp(), ti);
                    //  mRobotTracker.UpdateTurretError(ti);
    
                    // Call update function based on target
                    mTargetData.replace(targType, ti);
                  }
                });
    
          } catch (Exception Targ) {
            // Exception Thrown when Trying to retrieve values from json object
            // System.out.println("Packet Storing Exception: " + Targ.getMessage());
            return;
          }
        } catch (Exception e) {
          // Other Exception
          System.out.println("Parse Packet Exception: " + e.getMessage());
        }
      }
    
      /*
          Process every single packet
      */
      private void ProcessPacket() {
    
        while (true) {
          try {
            String PacketResult = PacketReciever.getPacket();
    
            // make sure this packet has size
            if (PacketResult.length() == 0) {
              continue; // null packet.. see ya!
            }
    
            try {
    
              // Pass call to a Runnable Object
              // this will execute in parallel
              executor.execute(
                  new Runnable() {
                    public void run() {
                      ParsePacket(PacketResult);
                    }
                  });
    
            } catch (Exception e) {
              System.out.println("Error: Cannot parse recieved UDP json data: " + e.toString());
              e.printStackTrace();
            }
          } catch (Exception e) {
    
          }
        }
      }

    // Creates the concurrent data structure
    // Stores default invalid values for each target
    private void initTargetStorage(){
        mTargetData = new ConcurrentHashMap<Constants.TargetType, TargetInfo>();
        for (Constants.TargetType target : Constants.TargetType.values()) {
            mTargetData.put(target, new TargetInfo());
        }
    }

    // set selected target
    public void setSelectedTarget(Constants.TargetType ttype){
      mSelectedTarget = ttype;
    }

    // Get TargetInfo for TargetType
    // The TargetInfo Object may not be valid
    public TargetInfo getTarget(Constants.TargetType ttype){
        return mTargetData.get(ttype);
    }

    // Gets a Target within parameter of seconds
    // if there is no target that recent, than null is returned
    public TargetInfo getTarget(Constants.TargetType ttype, double withinSeconds){
      TargetInfo t = mTargetData.get(ttype);

      // verify the target is within the threshold critera
      double currentSeconds = Timer.getFPGATimestamp();
      double deltaSeconds = currentSeconds - t.getTimestamp();

      // check if target is too old
      if(deltaSeconds > withinSeconds){
        // difference of seconds is greater than than difference
        t = null;
      }
      return t;
    }

    // Get Selected Target
    public TargetInfo getSelectedTarget(double withinSeconds){
      return getTarget(mSelectedTarget, withinSeconds);
    }    


}
