package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToClimb extends SequentialCommandGroup {

    private CommandSwerveDrivetrain swerve;

    public DriveToClimb(final CommandSwerveDrivetrain swerve, final int right) {
        this.swerve = swerve;
        this.addRequirements(this.swerve);

        this.addCommands(
                new DeferredCommand(
                        () -> {
                            final double currentY = this.swerve.getState().Pose.getY();

                            Command blueCmd;
                            Command redCmd;

                            // Blue alliance logic
                            // If Y < 3.75 and left (right == 0), do nothing
                            // If Y >= 3.75 and right (right == 1), do nothing
                            if ((right == 0 && currentY < 3.75) || (right == 1 && currentY >= 3.75)) {
                                blueCmd = new InstantCommand();
                            } else {
                                blueCmd = new CloseDriveToPose(
                                        swerve,
                                        this.adjustTargetYawForAlliance(
                                                this.addRobotCentricToFieldCentric(
                                                        SwerveConstants.BLUE_CLIMB_POSE,
                                                        right,
                                                        false),
                                                right,
                                                false));
                            }

                            // Red alliance logic (same conditions, mirrored Y)
                            // Field width is typically 8.21m for FRC
                            final double mirrorY = 8.21 - currentY;

                            if ((right == 0 && mirrorY < 3.75) || (right == 1 && mirrorY >= 3.75)) {
                                redCmd = new InstantCommand();
                            } else {
                                redCmd = new CloseDriveToPose(
                                        swerve,
                                        this.adjustTargetYawForAlliance(
                                                this.addRobotCentricToFieldCentric(
                                                        SwerveConstants.RED_CLIMB_POSE,
                                                        right,
                                                        true),
                                                right,
                                                true));
                            }

                            return new ConditionalAllianceCommand(blueCmd, redCmd);
                        },
                        this.getRequirements()));
    }

    private Pose2d adjustTargetYawForAlliance(final Pose2d targetPose, final int right, final boolean isRedAlliance) {
        Rotation2d rotation = targetPose.getRotation();

        // TODO THIS ASSUMES THAT YOUR CLIMBER IS ON THE LEFT SIDE OF YOUR ROBOT
        if (right == 0) {
            rotation = rotation.plus(Rotation2d.fromDegrees(180));
        } else if (right == 1) {
            rotation = rotation.plus(Rotation2d.fromDegrees(0));
        }

        // Then flip for red alliance if needed
        if (isRedAlliance) {
            rotation = rotation.plus(Rotation2d.fromDegrees(180));
        }

        return new Pose2d(
                targetPose.getX(),
                targetPose.getY(),
                rotation);
    }

    public Pose2d addRobotCentricToFieldCentric(final Pose2d robotPose, final int right, final boolean isRedAlliance) {
        double xOffset = Constants.SwerveConstants.getAdditions(right, 0);
        double yOffset = Constants.SwerveConstants.getAdditions(right, 1);

        // Flip the X and Y offsets for the red alliance
        if (isRedAlliance) {
            xOffset *= -1;
            yOffset *= -1;
        }

        final double newX = robotPose.getX() + (xOffset * Math.cos(robotPose.getRotation().getRadians())
                - yOffset * Math.sin(robotPose.getRotation().getRadians()));
        final double newY = robotPose.getY() + (xOffset * Math.sin(robotPose.getRotation().getRadians())
                + yOffset * Math.cos(robotPose.getRotation().getRadians()));
        return new Pose2d(newX, newY, robotPose.getRotation());
    }
}