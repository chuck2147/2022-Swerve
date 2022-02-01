package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.NTValue;
import frc.robot.PIDNTValue;
import frc.robot.constants.AutoConstants;
import frc.robot.motion.Trajectory;
import frc.robot.motion.TrajectoryPoint;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FollowPathCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  private final Trajectory trajectory;
  private double startTime = 0;
  private double endTime = 0;

  private final String pathName;

  private final PIDController pid_x = new PIDController(AutoConstants.kPController_X, AutoConstants.kIController_X, AutoConstants.kDController_X);
  private final PIDController pid_y = new PIDController(AutoConstants.kPController_Y, AutoConstants.kIController_Y, AutoConstants.kDController_Y);

  private final ProfiledPIDController rotationController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  private static final NetworkTableInstance nt = NetworkTableInstance.getDefault();

  //Since y and x use the same pid values other than F I will but P I and D into the sae PIDNTValue. 
  // private final PIDNTValue pid_PIDNT_x_y = new PIDNTValue(translation_kP, translation_kI, translation_kD, pid_x, "Follow Path pid_x and pid_y");
  // private final PIDNTValue pid_PIDNT_rotation = new PIDNTValue(rotation_kP, rotation_kI, rotation_kD, pid_rotation, "Follow Path pid_rotation");
  // private final NTValue pid_NT_translation_kF_x = new NTValue(translation_kF_x, "Translation_kF_x");
  // private final NTValue pid_NT_translation_kF_y = new NTValue(translation_kF_y, "Translation_kF_y");
  private static final NetworkTable pathFollowingTable = nt.getTable("/pathFollowing");
  private static final NetworkTable targetPoseTable = nt.getTable("/pathFollowing/target");
  private static final NetworkTableEntry targetXEntry = targetPoseTable.getEntry("x");
  //private final NTValue translations_kF_x_NtValue = new NTValue(translation_kF_x, "TranslationFeedForward_X");
  private static final NetworkTableEntry targetYEntry = targetPoseTable.getEntry("y");
  //private final NTValue translations_kF_y_NtValue = new NTValue(translation_kF_y, "TranslationFeedForward_Y");
  private static final NetworkTableEntry targetAngleEntry = targetPoseTable.getEntry("Angle");
  private static final NetworkTableEntry currentPathEntry = pathFollowingTable.getEntry("Current Path");

  private static boolean isFirstPath = true;

  private static boolean m_useFeedback = false;

  public static void onDisabled() {
    isFirstPath = true;
    currentPathEntry.setString("");
  }

  public FollowPathCommand(DrivetrainSubsystem drivetrain, String pathName) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    trajectory = Trajectory.fromJSON(pathName);
    this.pathName = pathName;

    rotationController.enableContinuousInput(AutoConstants.rotationMinInput, AutoConstants.rotationMaxInput);
  }

  @Override
  public void initialize() {
    System.out.println("Following path: " + pathName);
    startTime = Timer.getFPGATimestamp();
    var lastPoint = trajectory.points[trajectory.points.length - 1];
    endTime = startTime + lastPoint.time;
    if (isFirstPath) {
      var firstPoint = trajectory.points[0];
      drivetrain.resetPose(new Vector2d(firstPoint.x, firstPoint.y), Rotation2d.fromDegrees(firstPoint.angle));
    }

    isFirstPath = false;

    currentPathEntry.setValue(pathName);
  }

  @Override
  public void execute() {
    final var now = Timer.getFPGATimestamp();
    final var timeStamp = now - startTime;
    TrajectoryPoint beforePoint = null;
    TrajectoryPoint afterPoint = null;
    TrajectoryPoint betweenPoint = null;
    final var lastPoint = trajectory.points[trajectory.points.length - 1];

    if (timeStamp <= trajectory.points[0].time) {
        betweenPoint = trajectory.points[0];
    } else if (timeStamp >= lastPoint.time) {
        betweenPoint = lastPoint;
    } else {
        for (int i = 0; i < trajectory.points.length; i++) {
            var point = trajectory.points[i];
            if (timeStamp > point.time) {
                afterPoint = point;
                beforePoint = i > 0 ? trajectory.points[i - 1] : point;
            }
        }
        if (afterPoint == beforePoint) {
          betweenPoint = beforePoint;
        } else {
          var percent = (timeStamp - beforePoint.time) / (afterPoint.time - beforePoint.time);
          betweenPoint = TrajectoryPoint.createTrajectoryPointBetween(beforePoint, afterPoint, percent);
        }
    }

    targetXEntry.setValue(betweenPoint.x);
    targetYEntry.setValue(betweenPoint.y);
    targetAngleEntry.setValue(betweenPoint.angle);

    final var currentPose = drivetrain.getScaledPose();

    var feedForwardX = betweenPoint.velocity.x * AutoConstants.FeedForwardTranslation_X;
    var feedForwardY = betweenPoint.velocity.y * AutoConstants.FeedForwardTranslation_Y;    

    var setVelocityX = feedForwardX;
    var setVelocityY = feedForwardY;

    if (m_useFeedback) {
      var feedbackX = pid_x.calculate(currentPose.getX(), betweenPoint.x);    
      var feedbackY = pid_y.calculate(currentPose.getY(), betweenPoint.y);

      setVelocityX += feedbackX;
      setVelocityY += feedbackY;
    }
    
    System.out.println(betweenPoint.x + " " + betweenPoint.y);
    System.out.println(setVelocityX + " " + setVelocityY);

    final var rotationResult = rotationController.calculate(currentPose.getRotation().getRadians(), Rotation2d.fromDegrees(betweenPoint.angle).getRadians());

    drivetrain.drive(new ChassisSpeeds(setVelocityX, setVelocityY, rotationResult));
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() > endTime;
  }

  @Override
  public void end(boolean interrupted) {
      currentPathEntry.setString("");
  }

  // private ChassisSpeeds calculate(
  //     Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, Rotation2d angleRef) {
  //   // If this is the first run, then we need to reset the theta controller to the current pose's
  //   // heading.
  //   if (m_firstRun) {
  //     m_thetaController.reset(currentPose.getRotation().getRadians());
  //     m_firstRun = false;
  //   }

  //   // Calculate feedforward velocities (field-relative).
  //   double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
  //   double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();
  //   double thetaFF =
  //       m_thetaController.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());

  //   m_poseError = poseRef.relativeTo(currentPose);
  //   m_rotationError = angleRef.minus(currentPose.getRotation());

  //   if (!m_enabled) {
  //     return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
  //   }

  //   // Calculate feedback velocities (based on position error).
  //   double xFeedback = m_xController.calculate(currentPose.getX(), poseRef.getX());
  //   double yFeedback = m_yController.calculate(currentPose.getY(), poseRef.getY());

  //   // Return next output.
  //   return ChassisSpeeds.fromFieldRelativeSpeeds(
  //       xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
  // }

}
