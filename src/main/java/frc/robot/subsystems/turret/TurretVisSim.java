// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot.subsystems.Vision;
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
        ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

        double horizontalVel = Math.cos(angle.in(Radians)) * vel.in(MetersPerSecond);
        double verticalVel = Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond);
        double xVel =
                horizontalVel * Math.cos(turret.getCumulativeAngle());
        double yVel =
                horizontalVel * Math.sin(turret.getCumulativeAngle());

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        return new Translation3d(xVel, yVel, verticalVel);
    }

    public LinearVelocity getSimShooterVelo(){
        Pose3d target = vision.getHub3D();
        double dx = target.getX() - poseSupplier.get().getX();
        double dz = target.getZ() - poseSupplier.get().getZ();
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
        Pose3d robot = poseSupplier.get();

        Translation3d initialPosition = robot.getTranslation();
        FuelSim.getInstance().spawnFuel(initialPosition, launchVel(vel, angle));
    }

    public Command repeatedlyLaunchFuel(
            Supplier<LinearVelocity> velSupplier, Supplier<Angle> angleSupplier, Turret turret) {
        return turret.runOnce(() -> launchFuel(velSupplier.get(), angleSupplier.get()))
                .andThen(Commands.waitSeconds(0.25))
                .repeatedly();
    }

    public void updateFuel(LinearVelocity vel, Angle angle) {
        Translation3d trajVel = launchVel(vel, angle);
        for (int i = 0; i < trajectory.length; i++) {
            double t = i * 0.04;
            double x = trajVel.getX() * t + poseSupplier.get().getTranslation().getX();
            double y = trajVel.getY() * t + poseSupplier.get().getTranslation().getY();
            double z = trajVel.getZ() * t
                    - 0.5 * 9.81 * t * t
                    + poseSupplier.get().getTranslation().getZ();

            trajectory[i] = new Translation3d(x, y, z);
        }

        Logger.recordOutput("Turret/Trajectory", trajectory);
    }

    public void update3dPose(Angle azimuthAngle) {
        Logger.recordOutput("Turret/TurretPose", new Pose3d(0, 0, 0, new Rotation3d(0, 0, azimuthAngle.in(Radians))));
    }

    @Override
    public void simulationPeriodic() {
        updateFuel(getSimShooterVelo(), getSimShooterTheta());
        update3dPose(Degrees.of(turret.getCumulativeAngle()));
    }
    
}