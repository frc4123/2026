// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.turret.TrajectoryCalculator.ShotData;
import frc.robot.utils.Field;
import frc.robot.utils.FuelSim;
import frc.robot.utils.ShotCache;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class TurretVisSim extends SubsystemBase {
    private final Translation3d[] trajectory = new Translation3d[50];
    private final Supplier<Pose3d> poseSupplier;
    // private Supplier<ChassisSpeeds> fieldSpeedsSupplier;
    private static final int CAPACITY = 30;
    private int fuelStored = 3;
    private final Turret turret;

    public TurretVisSim(final Supplier<Pose3d> poseSupplier, final Turret turret) {
        this.poseSupplier = poseSupplier;
        // this.fieldSpeedsSupplier = fieldSpeedsSupplier;
        this.turret = turret;
    }

    private Translation3d launchVel(final LinearVelocity vel, final Angle angle) {
        final double elevationRad = angle.in(Units.Radians);
        final double turretFieldRad = Math.toRadians(this.turret.getFieldAngle());

        // CRITICAL LOGGING - AdvantageKit
        final double robotHeading =
                this.poseSupplier.get().getRotation().toRotation2d().getDegrees();
        final double cumulativeAngle = this.turret.getCumulativeAngle();

        Logger.recordOutput("LaunchFuel/RobotHeading", robotHeading);
        Logger.recordOutput("LaunchFuel/CumulativeAngle", cumulativeAngle);
        Logger.recordOutput("LaunchFuel/TurretFieldAngle", Math.toDegrees(turretFieldRad));

        final double horizontalVel = Math.cos(elevationRad) * vel.in(Units.MetersPerSecond);
        final double fieldXVel = horizontalVel * Math.cos(turretFieldRad);
        final double fieldYVel = horizontalVel * Math.sin(turretFieldRad);
        final double fieldZVel = Math.sin(elevationRad) * vel.in(Units.MetersPerSecond);

        final double actualDirection = Math.toDegrees(Math.atan2(fieldYVel, fieldXVel));
        Logger.recordOutput("LaunchFuel/LaunchDirection", actualDirection);
        Logger.recordOutput("LaunchFuel/FieldXVel", fieldXVel);
        Logger.recordOutput("LaunchFuel/FieldYVel", fieldYVel);

        return new Translation3d(fieldXVel, fieldYVel, fieldZVel);
    }

    private Pose3d getHubPose3D() {
        // Original behavior has this return blue if you ``aren't connected and are
        // red``
        if (!DriverStation.isDSAttached()) {
            return VisionConstants.BLUE_HUB;
        }
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return VisionConstants.BLUE_HUB;
        } else {
            return VisionConstants.RED_HUB;
        }
    }

    public LinearVelocity getSimShooterVelo() {
        // FIXED: Calculate distance from TURRET to target, not robot center
        final Pose3d target = this.getHubPose3D();
        final Translation3d turretPos = this.getTurretPosition();
        final double dx = target.getX() - turretPos.getX();
        final double dz = target.getZ() - turretPos.getZ();
        final double g = 9.81;
        final double speed =
                Math.sqrt(
                        g * dx * dx / (2 * Math.cos(45) * Math.cos(45) * (dx * Math.tan(45) - dz)));
        // Wrap speed in LinearVelocity type
        return Units.MetersPerSecond.of(speed);
    }

    public Angle getSimShooterTheta() {
        return Units.Degrees.of(45);
    }

    public boolean canIntake() {
        return this.fuelStored < TurretVisSim.CAPACITY;
    }

    public void intakeFuel() {
        this.fuelStored++;
    }

    public void launchFuel(final LinearVelocity vel, final Angle angle) {
        if (this.fuelStored == 0) return;
        this.fuelStored--;

        // FIXED: Launch from turret position, not robot center
        final Translation3d launchPos = this.getTurretPosition();
        FuelSim.getInstance().spawnFuel(launchPos, this.launchVel(vel, angle));
    }

    private Translation3d getTurretPosition() {
        final Pose3d robot = this.poseSupplier.get();
        final Rotation2d robotHeading = robot.getRotation().toRotation2d();

        // Turret offset in robot coordinates (2D for horizontal, separate Z)
        final Translation2d turretOffset2d =
                new Translation2d(
                        Constants.TurretConstants.OFFSET_X, Constants.TurretConstants.OFFSET_Y);

        // Rotate by robot heading
        final Translation2d rotatedOffset2d = turretOffset2d.rotateBy(robotHeading);

        // Convert to 3D with Z height
        return new Translation3d(
                robot.getX() + rotatedOffset2d.getX(),
                robot.getY() + rotatedOffset2d.getY(),
                robot.getZ() + Constants.TurretConstants.OFFSET_Z);
    }

    public Command repeatedlyLaunchFuel(
            final Supplier<LinearVelocity> velSupplier,
            final Supplier<Angle> angleSupplier,
            final Turret turret) {
        return turret.runOnce(() -> this.launchFuel(velSupplier.get(), angleSupplier.get()))
                .andThen(Commands.waitSeconds(0.25))
                .repeatedly();
    }

    public void updateFuel(final LinearVelocity vel, final Angle angle) {
        final Translation3d trajVel = this.launchVel(vel, angle);

        // FIXED: Get turret position for launch point
        final Translation3d turretPos = this.getTurretPosition();

        for (int i = 0; i < this.trajectory.length; i++) {
            final double t = i * 0.04;
            // FIXED: Use turret position as starting point
            final double x = trajVel.getX() * t + turretPos.getX();
            final double y = trajVel.getY() * t + turretPos.getY();
            final double z = trajVel.getZ() * t - 0.5 * 9.81 * t * t + turretPos.getZ();

            this.trajectory[i] = new Translation3d(x, y, z);
        }

        Logger.recordOutput("Turret/Trajectory", this.trajectory);
    }

    public void update3dPose(final Angle azimuthAngle) {
        // FIXED: Log actual turret pose in field coordinates
        final Translation3d turretPos = this.getTurretPosition();
        Logger.recordOutput(
                "Turret/TurretPose",
                new Pose3d(turretPos, new Rotation3d(0, 0, azimuthAngle.in(Units.Radians))));
    }

    public Translation3d getTurretTarget() {
        final double x = this.poseSupplier.get().getX();
        final double y = this.poseSupplier.get().getY();

        if (Field.isBlue()) {
            if (x < Constants.VisionConstants.BLUE_HUB.getX()) {
                // empty
            } else if (y >= 5.029) {
                // Top zone - depot
                return Constants.VisionConstants.blueDepotAim.getTranslation(); // Convert Pose2d to
                // Translation3d
            } else if (y > 4.044) {
                // Upper middle zone - left bump corner
                return Constants.VisionConstants.blueDepotAim.getTranslation();
            } else if (y > 3.059) {
                // Lower middle zone - right bump corner
                return Constants.VisionConstants.BLUE_AIM_THRESHOLD.getTranslation();
            } else {
                // Bottom zone - aim threshold
                return Constants.VisionConstants.BLUE_AIM_THRESHOLD.getTranslation();
            }
        } else if (Field.isRed()) {
            if (x > Constants.VisionConstants.RED_HUB.getX()) {
                return Constants.VisionConstants.redHubTranslation3d;
                // Past hub - match the Y zones from Turret.targetAngle()
            } else if (y >= 5.029) {
                // Top zone - aim threshold
                return Constants.VisionConstants.RED_AIM_THRESHOLD.getTranslation();
            } else if (y > 4.044) {
                // Upper middle zone - right bump corner
                return Constants.VisionConstants.RED_AIM_THRESHOLD.getTranslation();
            } else if (y > 3.059) {
                // Lower middle zone - left bump corner
                return Constants.VisionConstants.redDepotAim.getTranslation();
            } else {
                // Bottom zone - depot
                return Constants.VisionConstants.redDepotAim.getTranslation();
            }
        }

        return Constants.VisionConstants.blueHubTranslation3d;
    }

    public void updateFuelWithAzimuth(
            final LinearVelocity vel, final Angle elevationAngle, final double azimuthRadians) {
        final double elevationRad = elevationAngle.in(Units.Radians);

        // Use the PASSED azimuth instead of turret's current angle
        final double horizontalVel = Math.cos(elevationRad) * vel.in(Units.MetersPerSecond);
        final double fieldXVel = horizontalVel * Math.cos(azimuthRadians);
        final double fieldYVel = horizontalVel * Math.sin(azimuthRadians);
        final double fieldZVel = Math.sin(elevationRad) * vel.in(Units.MetersPerSecond);

        final Translation3d trajVel = new Translation3d(fieldXVel, fieldYVel, fieldZVel);
        final Translation3d turretPos = this.getTurretPosition();

        for (int i = 0; i < this.trajectory.length; i++) {
            final double t = i * 0.04;
            final double x = trajVel.getX() * t + turretPos.getX();
            final double y = trajVel.getY() * t + turretPos.getY();
            final double z = trajVel.getZ() * t - 0.5 * 9.81 * t * t + turretPos.getZ();

            this.trajectory[i] = new Translation3d(x, y, z);
        }

        Logger.recordOutput("Turret/Trajectory", this.trajectory);
        Logger.recordOutput("Turret/TrajectoryAzimuth", Math.toDegrees(azimuthRadians));
    }

    @Override
    public void simulationPeriodic() {
        final Translation3d target = this.getTurretTarget();
        ShotData calculatedShot;

        if (ShotCache.isPassingShot()) {

            calculatedShot =
                    TrajectoryCalculator.calculatePass(this.poseSupplier.get().toPose2d(), target);
            Logger.recordOutput("Turret/ShotMode", "PASS");
        } else {
            calculatedShot =
                    TrajectoryCalculator.iterativeMovingShotFromFunnelClearance(
                            this.poseSupplier.get().toPose2d(), new ChassisSpeeds(), target, 3);
            Logger.recordOutput("Turret/ShotMode", "HUB");
        }

        // Calculate the ACTUAL azimuth angle to the target
        final Translation3d turretPos = this.getTurretPosition();
        final double dx = target.getX() - turretPos.getX();
        final double dy = target.getY() - turretPos.getY();
        double azimuthToTarget = Math.atan2(dy, dx);

        // Wrap azimuth angle relative to ±360° of turret limits
        double azimuthDeg = Math.toDegrees(azimuthToTarget);
        while (azimuthDeg > TurretConstants.MECHANISM_MAX_RANGE * 360.0) azimuthDeg -= 360.0;
        while (azimuthDeg < TurretConstants.MECHANISM_MIN_RANGE * 360.0) azimuthDeg += 360.0;

        // Clamp to physical limits
        azimuthDeg =
                Math.max(
                        TurretConstants.MECHANISM_MIN_RANGE * 360.0,
                        Math.min(TurretConstants.MECHANISM_MAX_RANGE * 360.0, azimuthDeg));

        azimuthToTarget = Math.toRadians(azimuthDeg);

        // Update trajectory using the CALCULATED direction to target
        this.updateFuelWithAzimuth(
                calculatedShot.getExitVelocity(), calculatedShot.getHoodAngle(), azimuthToTarget);
        this.update3dPose(Units.Degrees.of(this.turret.getFieldAngle()));

        Logger.recordOutput("Turret/Shot", calculatedShot);
        Logger.recordOutput("Turret/TargetPosition", target);
        Logger.recordOutput("Turret/CalculatedAzimuth", Math.toDegrees(azimuthToTarget));
        Logger.recordOutput("Turret/Hood Angle", calculatedShot.getHoodAngle().in(Units.Degrees));
    }
}
