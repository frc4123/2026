package frc.robot.utils;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DecapitationHelper {

    private static CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    public static boolean closeToTrench() {

        double poseX = swerve.getState().Pose.getX();

        return (poseX > (4.63 - 0.75) && poseX < (4.63 + 0.75))
                || (poseX > (11.91 - 0.75) && poseX < (11.91 + 0.75));
    }
}
