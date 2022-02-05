package frc.robot.auto;

import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.Trajectory.State;

import java.util.List;
import java.util.ArrayList;
import java.util.Collections;

// Utility Functions for Autonomous
public class AutoUtil {

    /**
     * Returns a cloned, reversed list of the given states.
     * Each state is reconstructed with negative velocity and 
     * the correct timepoint for the following backwards
     */
    public static List<State> getInvertedStates(List<State> states){
        List<State> invertedStates = new ArrayList<State>(states);
        Collections.reverse(invertedStates);

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

    /**
     * Returns a Reversed Trajectory List
     * @param t
     * @return
     */
    public static Trajectory getReversedTrajectory(Trajectory t){
        List<State> reversedStates = getInvertedStates(t.getStates());
        Trajectory reversedTrajectory = new Trajectory(reversedStates);
        return reversedTrajectory;
    }
    
}
