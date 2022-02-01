package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class AutoConstants {

    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    
    // public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    // public static final double kMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 10;

    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    //public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI / 4;

    public static final double FeedForwardTranslation_X = -0.018;
    public static final double FeedForwardTranslation_Y = 0.025;

    public static final double kPController_X = 0.1;
    public static final double kIController_X = 0;    
    public static final double kDController_X = 0;

    public static final double kPController_Y = 0.1;
    public static final double kIController_Y = 0;    
    public static final double kDController_Y = 0;

    public static final double kPThetaController = 1;

    public static final double rotationMinInput = 0;
    public static final double rotationMaxInput = 2 * Math.PI;

    //public static final double rotationMinInput = -Math.PI;
    //public static final double rotationMaxInput = Math.PI;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
