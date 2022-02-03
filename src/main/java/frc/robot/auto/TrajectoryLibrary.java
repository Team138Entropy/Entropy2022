package frc.robot.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import  edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Filesystem;

import java.util.List;
import java.util.Collections;
import java.util.ArrayList;
import java.io.IOException;
import java.nio.file.Path;

// Functions for Trajectories
public class TrajectoryLibrary {
    private static TrajectoryLibrary mInstance;

    // Base Path of the Autonomous Paths
    String trajectoryJSONFolder = "trajectories/output";


    public static synchronized TrajectoryLibrary getInstance() {
        if (mInstance == null) {
          mInstance = new TrajectoryLibrary();
        }
        return mInstance;
     }
    
     private TrajectoryLibrary(){
      //load in trajectory files
      getTrajectoryJsonFiles();
     }

     // Returns all files in the Trajectoy JSON Folder
     private void getTrajectoryJsonFiles(){
      Path trajectoryFolderPath = getTrajectoryPathFolder();
      var files = trajectoryFolderPath.toFile().listFiles();

     }
    
     public Path getTrajectoryPathFolder(){
       return Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONFolder);
     }
    
     // Get Path of Trajectory File
     public Path getTrajectoryPath(String TrajectoryName){
       return Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONFolder + "/" + TrajectoryName);
     }

     // Trajectories
     public Trajectory getTrajectoryByName(String fileName){
       Trajectory traj = null;
       try {
         Path trajectoryPath = getTrajectoryPath(fileName);
         traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
       } catch (IOException ex) {
           System.out.println("Unable to open trajectory: " + fileName);
       }
       return traj;
     }

     // Inverts States
     public static List<State> getInvertedStates(List<State> states){
       List<State> invertedStates = new ArrayList<State>(states);
       for(int i = 0; i < invertedStates.size(); i++){
         State currState = invertedStates.get(i);
         State newState = new State(
          states.get(i).timeSeconds,
          currState.velocityMetersPerSecond * -1,
          currState.accelerationMetersPerSecondSq,
          currState.poseMeters,
          currState.curvatureRadPerMeter
         );
         invertedStates.set(i, newState);
       }
       return invertedStates;
     }

     // Creates a reversed Trajectory
     public Trajectory getReversedTrajectory(Trajectory traj){
      List<State> invertedStates = getInvertedStates(traj.getStates());
      return new Trajectory(invertedStates);
     }


     public Trajectory getBasicTrajectory(){
      Trajectory traj = null;
      String fileName = "hardbolt2.wpilib.json";
      try {
        Path trajectoryPath = getTrajectoryPath(fileName);
        traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
          System.out.println("Unable to open trajectory: " + fileName);
      }
      return traj;
     }
     public Trajectory get_Tarmac1_B2_trajectory(){
      Trajectory traj = null;
      String fileName = "Tarmac1_B2.wpilib.json";
      try {
        Path trajectoryPath = getTrajectoryPath(fileName);
        traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
          System.out.println("Unable to open trajectory: " + fileName);
      }
      return traj;
     }
     public Trajectory get_B2_B3_trajectory(){
      Trajectory traj = null;
      String fileName = "B2_B3.wpilib.json";
      try {
        Path trajectoryPath = getTrajectoryPath(fileName);
        traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
          System.out.println("Unable to open trajectory: " + fileName);
      }
      return traj;
     }
     public Trajectory get_B3_Tarmac2_trajectory(){
      Trajectory traj = null;
      String fileName = "B3_Tarmac2.wpilib.json";
      try {
        Path trajectoryPath = getTrajectoryPath(fileName);
        traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
          System.out.println("Unable to open trajectory: " + fileName);
      }
      return traj;
     }
     public Trajectory get_Tarmac1_B2_Backwords(){
      Trajectory traj = null;
      String fileName = "Tarmac1_B2.Backwords.wpilib.json";
      try {
        Path trajectoryPath = getTrajectoryPath(fileName);
        traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
          System.out.println("Unable to open trajectory: " + fileName);
      }
      return traj;
     }
     public Trajectory get_B2_Tarmac1_trajectory(){
      Trajectory traj = null;
      String fileName = "B2_Tarmac1.wpilib.json";
      try {
        Path trajectoryPath = getTrajectoryPath(fileName);
        traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
          System.out.println("Unable to open trajectory: " + fileName);
      }
      return traj;
     }
     public Trajectory get_DEMO(){
      Trajectory traj = null;
      String fileName = "DEMO3.wpilib.json";
      try {
        Path trajectoryPath = getTrajectoryPath(fileName);
        traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
          System.out.println("Unable to open trajectory: " + fileName);
      }
      return traj;
     }
     
     
}


