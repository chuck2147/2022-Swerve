package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowPathCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveForward extends SequentialCommandGroup {
    private final DrivetrainSubsystem m_driveSubsystem;

    public DriveForward(DrivetrainSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        addRequirements(m_driveSubsystem);

        addCommands(
                // Drive from the starting line to the aiming spot while spinning up the shooter
                new FollowPathCommand(driveSubsystem, "Drive forward"));
    }
}