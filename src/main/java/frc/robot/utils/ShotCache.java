package frc.robot.utils;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.TurretCalculator;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;

public class ShotCache {

    private static ShotData cachedShot;
    private static final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    public static boolean isPassingShot() {
        double robotX = swerve.getState().Pose.getX();
        
        if(Field.isBlue()) {
            return robotX > Constants.VisionConstants.blueHub.getX();
        } else if(Field.isRed()) {
            return robotX < Constants.VisionConstants.redHub.getX();
        }
        
        return false;
    }
    
    public static void update() {
        if(!isPassingShot()) {
            cachedShot = TurretCalculator.iterativeMovingShotFromFunnelClearance(
                swerve.getState().Pose,
                swerve.getState().Speeds,
                Target.getTarget(),
                7 //was 3
            );
        } else {
            cachedShot = TurretCalculator.iterativeMovingPass(
                swerve.getState().Pose,
                swerve.getState().Speeds,
                Target.getTarget(),
                7 // was 3
            );
        }
        
    }

    public static ShotData get() {
        return cachedShot;
    }
}