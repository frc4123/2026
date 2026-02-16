// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.IntakeRollerConstants.BASE_VEL;
import static frc.robot.Constants.IntakeRollerConstants.VEL_MULTIPLIER;
import static frc.robot.Constants.IntakeRollerConstants.VEL_POWER;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.FieldConstants;

/** Add your docs here. */
public class TurretCalculator {

    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        Translation2d turretPos =
        robot.getTranslation().plus(
            Constants.TurretConstants.turretOffset.rotateBy(robot.getRotation())
        );

    return Meters.of(turretPos.getDistance(target.toTranslation2d()));
    }

    // see https://www.desmos.com/geometry/l4edywkmha 
    public static Angle calculateAngleFromVelocity(Pose2d robot, LinearVelocity velocity, Translation3d target) {
        double g = MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond);
        double vel = velocity.in(InchesPerSecond);
        double x_dist = getDistanceToTarget(robot, target).in(Inches);
        double y_dist = target.getMeasureZ()
                .minus(Meters.of(Constants.TurretConstants.offsetZ))
                .in(Inches);
        double angle = Math.atan(
                ((vel * vel) + Math.sqrt(Math.pow(vel, 4) - g * (g * x_dist * x_dist + 2 * y_dist * vel * vel)))
                        / (g * x_dist));

         // Clamp to physical constraints
        Angle calculatedAngle = Radians.of(angle);
        if (calculatedAngle.lt(HoodConstants.MIN_HOOD_ANGLE)) {
            return HoodConstants.MIN_HOOD_ANGLE;
        } else if (calculatedAngle.gt(HoodConstants.MAX_HOOD_ANGLE)) {
            return HoodConstants.MAX_HOOD_ANGLE;
        }

        return Radians.of(angle);
    }

    // calculates how long it will take for a projectile to travel a set distance given its initial velocity and angle
    public static Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
        double vel = exitVelocity.in(MetersPerSecond);
        double angle = hoodAngle.in(Radians);
        double dist = distance.in(Meters);
        return Seconds.of(dist / (vel * Math.cos(angle)));
    }

    public static AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
        return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters));
    }

    public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
        return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters));
    }

    // Move a target a set time in the future along a velocity defined by fieldSpeeds
    public static Translation3d predictTargetPos(Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
        double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
        double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);

        return new Translation3d(predictedX, predictedY, target.getZ());
    }

    // Custom velocity ramp meant to minimize how fast the flywheels have to change speed
    public static LinearVelocity scaleLinearVelocity(Distance distanceToTarget) {
        double velocity =
                BASE_VEL.in(InchesPerSecond) + VEL_MULTIPLIER * Math.pow(distanceToTarget.in(Inches), VEL_POWER);
        return InchesPerSecond.of(velocity);
    }

    // see https://www.desmos.com/calculator/ezjqolho6g
    public static ShotData calculateShotFromFunnelClearance(
            Pose2d robot, Translation3d actualTarget, Translation3d predictedTarget) {
        double x_dist = getDistanceToTarget(robot, predictedTarget).in(Inches);
        double y_dist = predictedTarget
                .getMeasureZ()
                .minus(Meters.of(Constants.TurretConstants.offsetZ))
                .in(Inches);
        double g = 386;
        double r = FieldConstants.FUNNEL_RADIUS.in(Inches)
                * x_dist
                / getDistanceToTarget(robot, actualTarget).in(Inches);
        double h = FieldConstants.FUNNEL_HEIGHT.plus(Constants.TurretConstants.DISTANCE_ABOVE_FUNNEL).in(Inches);
        double A1 = x_dist * x_dist;
        double B1 = x_dist;
        double D1 = y_dist;
        double A2 = -x_dist * x_dist + (x_dist - r) * (x_dist - r);
        double B2 = -r;
        double D2 = h;
        double Bm = -B2 / B1;
        double A3 = Bm * A1 + A2;
        double D3 = Bm * D1 + D2;
        double a = D3 / A3;
        double b = (D1 - A1 * a) / B1;
        double theta = Math.atan(b);
        double v0 = Math.sqrt(-g / (2 * a * (Math.cos(theta)) * (Math.cos(theta))));

        // Clamp angle to physical constraints
        Angle calculatedAngle = Radians.of(theta);
        boolean wasClamped = false;
        
        if (calculatedAngle.lt(HoodConstants.MIN_HOOD_ANGLE)) {
            calculatedAngle = HoodConstants.MIN_HOOD_ANGLE;
            wasClamped = true;
        } else if (calculatedAngle.gt(HoodConstants.MAX_HOOD_ANGLE)) {
            calculatedAngle = HoodConstants.MAX_HOOD_ANGLE;
            wasClamped = true;
        }
        
        // If we clamped, recalculate velocity for the new angle
        double finalV0;
        if (wasClamped) {
            double clampedTheta = calculatedAngle.in(Radians);
            // Standard projectile motion: solve for v0 given angle, x_dist, y_dist
            double cosTheta = Math.cos(clampedTheta);
            double sinTheta = Math.sin(clampedTheta);
            double tanTheta = sinTheta / cosTheta;
            
            // v0 = sqrt(g * x_dist^2 / (2 * cos^2(θ) * (x_dist * tan(θ) - y_dist)))
            double numerator = g * x_dist * x_dist;
            double denominator = 2 * cosTheta * cosTheta * (x_dist * tanTheta - y_dist);
            
            if (denominator > 0) {
                finalV0 = Math.sqrt(numerator / denominator);
            } else {
                // Shot physically impossible - mark as invalid or use max velocity
                finalV0 = Double.NaN; // Or some max velocity constant
            }
        } else {
            finalV0 = v0;
        }
        
        // ShotData constructor expects: (double exitVelocity_in/s, double hoodAngle_radians, Translation3d target)
        return new ShotData(InchesPerSecond.of(finalV0).in(MetersPerSecond), calculatedAngle.in(Radians), predictedTarget);
    }

    public static ShotData calculatePass(Pose2d robot, Translation3d target) {
        double x_dist = getDistanceToTarget(robot, target).in(Inches);
        double y_dist = target.getMeasureZ()
                .minus(Meters.of(Constants.TurretConstants.offsetZ))
                .in(Inches);
        double g = 386; // inches/s²
        
        // Use a fixed, comfortable velocity for passing (tune this!)
        double v0 = 200.0; // inches/second - adjust based on testing
        
        // Solve for angle: θ = atan((v² ± sqrt(v⁴ - g(gx² + 2yv²))) / (gx))
        double discriminant = Math.pow(v0, 4) - g * (g * x_dist * x_dist + 2 * y_dist * v0 * v0);
        
        if (discriminant < 0) {
            // Shot impossible at this velocity, increase it
            v0 = Math.sqrt(g * Math.sqrt(x_dist * x_dist + y_dist * y_dist));
            discriminant = Math.pow(v0, 4) - g * (g * x_dist * x_dist + 2 * y_dist * v0 * v0);
        }
        
        // Use the lower arc (+ in the formula gives higher arc)
        double theta = Math.atan(
            ((v0 * v0) + Math.sqrt(discriminant)) / (g * x_dist)
        );
        
        // Clamp angle to physical constraints
        Angle calculatedAngle = Radians.of(theta);
        if (calculatedAngle.lt(HoodConstants.MIN_HOOD_ANGLE)) {
            calculatedAngle = HoodConstants.MIN_HOOD_ANGLE;
        } else if (calculatedAngle.gt(HoodConstants.MAX_HOOD_ANGLE)) {
            calculatedAngle = HoodConstants.MAX_HOOD_ANGLE;
        }
        
        return new ShotData(InchesPerSecond.of(v0).in(MetersPerSecond), 
                        calculatedAngle.in(Radians), target);
    }

    // use an iterative lookahead approach to determine shot parameters for a moving robot
    public static ShotData iterativeMovingShotFromFunnelClearance(
            Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
        // Perform initial estimation (assuming unmoving robot) to get time of flight estimate
        ShotData shot = calculateShotFromFunnelClearance(robot, target, target);
        Distance distance = getDistanceToTarget(robot, target);
        Time timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), distance);
        Translation3d predictedTarget = target;

        // Iterate the process, getting better time of flight estimations and updating the predicted target accordingly
        for (int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            shot = calculateShotFromFunnelClearance(robot, target, predictedTarget);
            timeOfFlight = calculateTimeOfFlight(
                    shot.getExitVelocity(), shot.getHoodAngle(), getDistanceToTarget(robot, predictedTarget));
        }

        return shot;
    }

    public record ShotData(double exitVelocity, double hoodAngle, Translation3d target) {
        public ShotData(LinearVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
            this(exitVelocity.in(MetersPerSecond), hoodAngle.in(Radians), target);
        }

        public LinearVelocity getExitVelocity() {
            return MetersPerSecond.of(this.exitVelocity);
        }

        public Angle getHoodAngle() {
            return Radians.of(this.hoodAngle);
        }

        public Translation3d getTarget() {
            return this.target;
        }
    }
}