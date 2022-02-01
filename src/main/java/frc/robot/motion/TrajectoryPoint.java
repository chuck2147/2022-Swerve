package frc.robot.motion;

import edu.wpi.first.wpilibj.drive.Vector2d;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

@JsonIgnoreProperties(ignoreUnknown = true)
public class TrajectoryPoint {
    // This is capital Double so they get initialized to null, so the tests fail if
    // they are missing from the JSON parsing
    public Double x, y, heading, angularVelocity, time, angle;
    public Vector2d velocity;

    public static TrajectoryPoint createTrajectoryPointBetween(TrajectoryPoint before, TrajectoryPoint after,
            double percentage) {
        var newPoint = new TrajectoryPoint();
        newPoint.x = lerp(before.x, after.x, percentage);
        newPoint.y = lerp(before.y, after.y, percentage);
        newPoint.heading = lerp(before.heading, after.heading, percentage);
        newPoint.velocity = new Vector2d(lerp(before.velocity.x, after.velocity.x, percentage),
                lerp(before.velocity.y, after.velocity.y, percentage));
        newPoint.angularVelocity = lerp(before.angularVelocity, after.angularVelocity, percentage);
        newPoint.time = lerp(before.time, after.time, percentage);
        newPoint.angle = lerp(before.angle, after.angle, percentage);

        return newPoint;

    }
    public static double lerp(double start, double end, double t) {
		return start + (end - start) * t;
	}
}