package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Target {

    private static CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    public static Translation3d getTarget(){

        double x = swerve.getState().Pose.getX();
        double y = swerve.getState().Pose.getY();

        if(Field.isBlue()) {
            if(x < Constants.VisionConstants.blueHub.getX()){
                return Constants.VisionConstants.blueHubTranslation3d;
            // Past hub - match the Y zones from Turret.targetAngle()
            } else if (y >= 5.029) {
                // Top zone - depot
                return Constants.VisionConstants.blueDepot.getTranslation(); // blueDepot   Convert Pose2d to Translation3d
            } else if (y > 4.044) {
                // Upper middle zone - left bump corner
                return Constants.VisionConstants.blueDepot.getTranslation(); //blueLeftBumpCorner.
            } else if (y > 3.059) {
                // Lower middle zone - right bump corner
                return Constants.VisionConstants.blueAimThreshold.getTranslation(); //blueRIghtBumpCorner
            } else {
                // Bottom zone - aim threshold
                return Constants.VisionConstants.blueAimThreshold.getTranslation(); //blueAimThreshold
            }
        }
        else if(Field.isRed()) {
            if(x > Constants.VisionConstants.redHub.getX()){
                return Constants.VisionConstants.redHubTranslation3d;
            // Past hub - match the Y zones from Turret.targetAngle()
            } else if (y >= 5.029) {
                // Top zone - aim threshold
                return Constants.VisionConstants.redAimThreshold.getTranslation(); // redAimThreshold
            } else if (y > 4.044) {
                // Upper middle zone - right bump corner
                return Constants.VisionConstants.redAimThreshold.getTranslation(); // redRightBumpCorner
            } else if (y > 3.059) {
                // Lower middle zone - left bump corner
                return Constants.VisionConstants.redDepot.getTranslation(); //redLeftBumpCorner
            } else {
                // Bottom zone - depot
                return Constants.VisionConstants.redDepot.getTranslation(); //redDepot
            }
        }
    
        return Constants.VisionConstants.blueHubTranslation3d;
    }
}
