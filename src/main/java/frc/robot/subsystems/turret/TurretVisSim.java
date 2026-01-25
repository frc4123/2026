// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.utils.FuelSim;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class TurretVisSim extends SubsystemBase{
    private Translation3d[] trajectory = new Translation3d[50];
    private Supplier<Pose3d> poseSupplier;
    private Supplier<ChassisSpeeds> fieldSpeedsSupplier;
    private final int CAPACITY = 30;
    private int fuelStored = 8;
    private Vision vision;
    private Turret turret;

    public TurretVisSim(Supplier<Pose3d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier, Vision vision, Turret turret) {
        this.poseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;
        this.vision = vision;
        this.turret = turret;
    } 

    private Translation3d launchVel(LinearVelocity vel, Angle angle) {
        Pose3d robot = poseSupplier.get();
        ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
        
        double elevationRad = angle.in(Radians);
        double turretYawRad = Math.toRadians(turret.getCumulativeAngle());
        
        // Projectile velocity in ROBOT frame (before rotation compensation)
        double robotXVel = Math.cos(elevationRad) * Math.cos(turretYawRad) * vel.in(MetersPerSecond);
        double robotYVel = Math.cos(elevationRad) * Math.sin(turretYawRad) * vel.in(MetersPerSecond);
        double robotZVel = Math.sin(elevationRad) * vel.in(MetersPerSecond);
        
        // Rotate projectile velocity to FIELD frame by robot heading
        double robotHeadingRad = robot.getRotation().toRotation2d().getRadians();
        double fieldXVel = robotXVel * Math.cos(robotHeadingRad) - robotYVel * Math.sin(robotHeadingRad);
        double fieldYVel = robotXVel * Math.sin(robotHeadingRad) + robotYVel * Math.cos(robotHeadingRad);
        
        // ONLY add robot TRANSLATION velocity (not rotation effects)
        // The turret already compensates for rotation by tracking the target
        //fieldXVel += fieldSpeeds.vxMetersPerSecond;
        //fieldYVel += fieldSpeeds.vyMetersPerSecond;
        
        return new Translation3d(fieldXVel, fieldYVel, robotZVel);
    }

    public LinearVelocity getSimShooterVelo(){
        // FIXED: Calculate distance from TURRET to target, not robot center
        Pose3d target = vision.getHub3D();
        Translation3d turretPos = getTurretPosition();
        double dx = target.getX() - turretPos.getX();
        double dz = target.getZ() - turretPos.getZ();
        double g = 9.81;
        double speed = Math.sqrt(g * dx * dx / (2 * Math.cos(45) * Math.cos(45) * (dx * Math.tan(45) - dz)));
        // Wrap speed in LinearVelocity type
        LinearVelocity vel = MetersPerSecond.of(speed);

        return vel;
    }

    public Angle getSimShooterTheta(){
        Angle theta = Degrees.of(45);
        return theta;
    }

    public boolean canIntake() {
        return fuelStored < CAPACITY;
    }

    public void intakeFuel() {
        fuelStored++;
    }

    public void launchFuel(LinearVelocity vel, Angle angle) {
        if (fuelStored == 0) return;
        fuelStored--;
        
        // FIXED: Launch from turret position, not robot center
        Translation3d launchPos = getTurretPosition();
        FuelSim.getInstance().spawnFuel(launchPos, launchVel(vel, angle));
    }

    private Translation3d getTurretPosition() {
        Pose3d robot = poseSupplier.get();
        Rotation2d robotHeading = robot.getRotation().toRotation2d();
        
        // Turret offset in robot coordinates (2D for horizontal, separate Z)
        Translation2d turretOffset2d = new Translation2d(
            Constants.Turret.offsetX,
            Constants.Turret.offsetY
        );
        
        // Rotate by robot heading
        Translation2d rotatedOffset2d = turretOffset2d.rotateBy(robotHeading);
        
        // Convert to 3D with Z height
        return new Translation3d(
            robot.getX() + rotatedOffset2d.getX(),
            robot.getY() + rotatedOffset2d.getY(),
            robot.getZ() + Constants.Turret.offsetZ
        );
    }

    public Command repeatedlyLaunchFuel(
            Supplier<LinearVelocity> velSupplier, Supplier<Angle> angleSupplier, Turret turret) {
        return turret.runOnce(() -> launchFuel(velSupplier.get(), angleSupplier.get()))
                .andThen(Commands.waitSeconds(0.25))
                .repeatedly();
    }

    public void updateFuel(LinearVelocity vel, Angle angle) {
        Translation3d trajVel = launchVel(vel, angle);
        
        // FIXED: Get turret position for launch point
        Translation3d turretPos = getTurretPosition();
        
        for (int i = 0; i < trajectory.length; i++) {
            double t = i * 0.04;
            // FIXED: Use turret position as starting point
            double x = trajVel.getX() * t + turretPos.getX();
            double y = trajVel.getY() * t + turretPos.getY();
            double z = trajVel.getZ() * t - 0.5 * 9.81 * t * t + turretPos.getZ();

            trajectory[i] = new Translation3d(x, y, z);
        }

        Logger.recordOutput("Turret/Trajectory", trajectory);
    }

    public void update3dPose(Angle azimuthAngle) {
        // FIXED: Log actual turret pose in field coordinates
        Translation3d turretPos = getTurretPosition();
        Logger.recordOutput("Turret/TurretPose", 
            new Pose3d(turretPos, new Rotation3d(0, 0, azimuthAngle.in(Radians))));
    }

    public Translation3d getTurretTarget(){
        if(vision.isBlue() &&  poseSupplier.get().getX() < Constants.FieldConstants.HUB_BLUE.getX()){
            return Constants.VisionConstants.blueHubTranslation3d;
        }
        else if(vision.isRed() && poseSupplier.get().getX() > Constants.FieldConstants.HUB_RED.getX()) {
            return Constants.VisionConstants.redHubTranslation3d;
        }
        return Constants.VisionConstants.blueHubTranslation3d;

    }

    @Override
    public void simulationPeriodic() {
        ShotData calculatedShot = TurretCalculator.iterativeMovingShotFromFunnelClearance(
            poseSupplier.get().toPose2d(), 
            new ChassisSpeeds(), 
            getTurretTarget(), 
            3 // or whatever LOOKAHEAD_ITERATIONS you use
        );
        updateFuel(calculatedShot.getExitVelocity(), calculatedShot.getHoodAngle());
        update3dPose(Degrees.of(turret.getCumulativeAngle()));
        // Use the same calculation method as the other Turret.java
        
        
        // Log the shot data 
        Logger.recordOutput("Turret/Shot", calculatedShot);
    }
    
}