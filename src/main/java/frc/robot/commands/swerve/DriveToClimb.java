package frc.robot.commands.swerve;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToClimb extends SequentialCommandGroup {

    private CommandSwerveDrivetrain swerve;

    public DriveToClimb(CommandSwerveDrivetrain swerve, int right) {
        this.swerve = swerve;
        addRequirements(this.swerve);
    
        addCommands(
            new DeferredCommand(
                () -> (new ConditionalAllianceCommand(
                    // Blue alliance command
                    new CloseDriveToPose(
                        swerve,
                        adjustTargetYawForAlliance(
                            addRobotCentrictoFieldCentric(
                                SwerveConstants.BLUE_CLIMB_POSE,
                                right, // int parameter
                                false // false for blue alliance
                            ),
                            right,
                            false // false for blue alliance
                        )
                    ),
                    // Red alliance command
                    new CloseDriveToPose(
                        swerve,
                        adjustTargetYawForAlliance(
                            addRobotCentrictoFieldCentric(
                                SwerveConstants.RED_CLIMB_POSE,
                                right, // int parameter
                                true // true for red alliance
                            ),
                            right,
                            true // true for red alliance
                        )
                    )
                )),
                getRequirements()
            )
        );
    }
    
    private Pose2d adjustTargetYawForAlliance(Pose2d targetPose, int right, boolean isRedAlliance) {
        Rotation2d rotation = targetPose.getRotation();
        
        // TODO THIS ASSUMES THAT YOUR CLIMBER IS ON THE LFFT SIDE OF YOUR ROBOT
        if (right == 0) {
            rotation = rotation.plus(Rotation2d.fromDegrees(180)); // Example: add 30 degrees if right is 1
        } else if (right == 1) {
            rotation = rotation.plus(Rotation2d.fromDegrees(0)); // Example: subtract 30 degrees if right is 2
        }
        // Add more conditions as needed for other right values
        
        // Then flip for red alliance if needed
        if (isRedAlliance) {
            rotation = rotation.plus(Rotation2d.fromDegrees(180));
        }
        
        return new Pose2d(
            targetPose.getX(),
            targetPose.getY(),
            rotation
        );
    }

    public Pose2d addRobotCentrictoFieldCentric(Pose2d robotPose, int right, boolean isRedAlliance) {
        double xOffset = Constants.SwerveConstants.ADDITIONS[right][0];
        double yOffset = Constants.SwerveConstants.ADDITIONS[right][1];
    
        // Flip the X and Y offsets for the red alliance
        if (isRedAlliance) {
            xOffset *= -1;
            yOffset *= -1; // Flip the Y offset as well
        }
    
        double newX = robotPose.getX() + (xOffset * Math.cos(robotPose.getRotation().getRadians()) - yOffset * Math.sin(robotPose.getRotation().getRadians()));
        double newY = robotPose.getY() + (xOffset * Math.sin(robotPose.getRotation().getRadians()) + yOffset * Math.cos(robotPose.getRotation().getRadians()));
        Pose2d calculated = new Pose2d(newX, newY, robotPose.getRotation());
        return calculated;
    }
}