package frc.robot.motion;

import java.io.IOException;

public class Trajectory {
    public String name;
    public TrajectoryPoint[] points;

    public static Trajectory fromJSON(String trajectoryName) {
        try {
            return TrajectoryReader.getTrajectory(trajectoryName);
        } catch (IOException e) {
            System.err.println("Unable to load trajectory: " + trajectoryName);
        }
        return null;
    }
}