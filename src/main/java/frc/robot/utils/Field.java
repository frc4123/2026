// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.utils;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;

/**
 * Contains various field dimensions and useful reference points. All units are
 * in meters and poses
 * have a blue alliance origin.
 */
public class Field {
        private Field() {
                /* This utility class should not be instantiated */
        }

        @Getter
        public static final double FIELD_LENGTH = Units.inchesToMeters(651.2);
        @Getter
        private static final double HALF_LENGTH = FIELD_LENGTH / 2.0;
        @Getter
        public static final double FIELD_WIDTH = Units.inchesToMeters(317.7);
        @Getter
        private static final double HALF_WIDTH = FIELD_WIDTH / 2.0;

        @Getter
        private static final Pose2d centerField = new Pose2d(HALF_LENGTH, HALF_WIDTH, new Rotation2d());

        @Getter
        public static final double STARTING_LINE_X = Units.inchesToMeters(299.438); // Measured from the inside of
                                                                                    // starting
                                                                                    // line

        public static final double TAG_26_X = Units.inchesToMeters(158.61);

        public static class BlueHub {
                private BlueHub() {
                        /* This utility class should not be instantiated */
                }

                public static final double width = Units.inchesToMeters(47.0);
                public static final double totalHeight = Units.inchesToMeters(72.0);
                public static final double innerOpeningWidth = Units.inchesToMeters(41.7);
                public static final double innerOpeningHeight = Units.inchesToMeters(56.5);

                public static final double centerX = TAG_26_X + width / 2.0;
                public static final double centerY = FIELD_WIDTH / 2.0;

                public static final Translation3d topCenter = new Translation3d(centerX, centerY, totalHeight);
                public static final Translation3d innerCenter = new Translation3d(centerX, centerY, innerOpeningHeight);
                public static final Translation2d nearLeftCorner = new Translation2d(TAG_26_X, centerY + width / 2.0);
                public static final Translation2d nearRightCorner = new Translation2d(TAG_26_X, centerY - width / 2.0);
                public static final Translation2d farLeftCorner = new Translation2d(TAG_26_X + width,
                                centerY + width / 2.0);
                public static final Translation2d farRightCorner = new Translation2d(TAG_26_X + width,
                                centerY - width / 2.0);
        }

        public static class BlueBumps {
                private BlueBumps() {
                        /* This utility class should not be instantiated */
                }

                public static final double width = Units.inchesToMeters(73.0);
                public static final double depth = Units.inchesToMeters(44.4);
                public static final double height = Units.inchesToMeters(6.513);
        }

        public static class LeftBlueBump {
                private LeftBlueBump() {
                        /* This utility class should not be instantiated */
                }

                public static final Translation2d nearLeftCorner = new Translation2d(
                                BlueHub.centerX - BlueBumps.depth / 2.0,
                                FIELD_WIDTH - BlueTrench.openingWidth);

                public static final Translation2d nearRightCorner = BlueHub.nearLeftCorner;

                public static final Translation2d farLeftCorner = new Translation2d(
                                BlueHub.centerX + BlueBumps.depth / 2.0,
                                FIELD_WIDTH - BlueTrench.openingWidth);

                public static final Translation2d farRightCorner = BlueHub.farLeftCorner;

                public static final Translation2d centerPose = new Translation2d(
                                (nearLeftCorner.getX() + nearRightCorner.getX() + farLeftCorner.getX()
                                                + farRightCorner.getX()) / 4.0,
                                (nearLeftCorner.getY() + nearRightCorner.getY() + farLeftCorner.getY()
                                                + farRightCorner.getY()) / 4.0);
        }

        public static class RightBlueBump {
                private RightBlueBump() {
                        /* This utility class should not be instantiated */
                }

                public static final Translation2d nearLeftCorner = BlueHub.nearRightCorner;

                public static final Translation2d nearRightCorner = new Translation2d(
                                BlueHub.centerX - BlueBumps.depth / 2.0,
                                BlueTrench.openingWidth);

                public static final Translation2d farLeftCorner = BlueHub.farRightCorner;

                public static final Translation2d farRightCorner = new Translation2d(
                                BlueHub.centerX + BlueBumps.depth / 2.0,
                                BlueTrench.openingWidth);

                public static final Translation2d centerPose = new Translation2d(
                                (nearLeftCorner.getX() + nearRightCorner.getX() + farLeftCorner.getX()
                                                + farRightCorner.getX()) / 4.0,
                                (nearLeftCorner.getY() + nearRightCorner.getY() + farLeftCorner.getY()
                                                + farRightCorner.getY()) / 4.0);
        }

        public static class BlueTrench {
                private BlueTrench() {
                        /* This utility class should not be instantiated */
                }

                public static final double width = Units.inchesToMeters(65.65);
                public static final double depth = Units.inchesToMeters(47.0);
                public static final double height = Units.inchesToMeters(40.25);
                public static final double openingWidth = Units.inchesToMeters(50.34);
                public static final double openingHeight = Units.inchesToMeters(22.25);
        }

        public static class LeftBlueTrench {
                private LeftBlueTrench() {
                        /* This utility class should not be instantiated */
                }

                public static final Translation3d openingTopLeft = new Translation3d(BlueHub.centerX, FIELD_WIDTH,
                                BlueTrench.openingHeight);

                public static final Translation3d openingTopRight = new Translation3d(
                                BlueHub.centerX,
                                FIELD_WIDTH - BlueTrench.openingWidth,
                                BlueTrench.openingHeight);
        }

        public static class RightBlueTrench {
                private RightBlueTrench() {
                        /* This utility class should not be instantiated */
                }

                public static final Translation3d openingTopLeft = new Translation3d(BlueHub.centerX,
                                BlueTrench.openingWidth,
                                BlueTrench.openingHeight);

                public static final Translation3d openingTopRight = new Translation3d(BlueHub.centerX, 0.0,
                                BlueTrench.openingHeight);
        }

        public static class BlueTower {
                private BlueTower() {
                        /* This utility class should not be instantiated */
                }

                public static final double width = Units.inchesToMeters(49.25);
                public static final double depth = Units.inchesToMeters(45.0);
                public static final double height = Units.inchesToMeters(78.25);
                public static final double innerOpeningWidth = Units.inchesToMeters(32.25);
                public static final double uprightHeight = Units.inchesToMeters(72.1);

                public static final double lowRungZ = Units.inchesToMeters(27.0);
                public static final double midRungZ = Units.inchesToMeters(45.0);
                public static final double highRungZ = Units.inchesToMeters(63.0);

                public static final double tag31Y = Units.inchesToMeters(FIELD_WIDTH / 2);

                // Fixed X location
                public static final double frontFaceX = Units.inchesToMeters(43.51);

                // Reference points
                public static final Translation2d center = new Translation2d(frontFaceX, tag31Y);

                public static final Translation2d leftUpright = new Translation2d(
                                frontFaceX,
                                tag31Y + innerOpeningWidth / 2.0 + Units.inchesToMeters(0.75));

                public static final Translation2d rightUpright = new Translation2d(
                                frontFaceX,
                                tag31Y - innerOpeningWidth / 2.0 - Units.inchesToMeters(0.75));
        }

        public static class BlueDepot {
                private BlueDepot() {
                        /* This utility class should not be instantiated */
                }

                public static final double width = Units.inchesToMeters(42.0);
                public static final double depth = Units.inchesToMeters(27.0);
                public static final double height = Units.inchesToMeters(1.125);
                public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

                public static final Translation3d center = new Translation3d(
                                depth,
                                FIELD_WIDTH / 2.0 + distanceFromCenterY,
                                height);

                public static final Translation3d leftCorner = new Translation3d(
                                depth,
                                FIELD_WIDTH / 2.0 + distanceFromCenterY + width / 2.0,
                                height);

                public static final Translation3d rightCorner = new Translation3d(
                                depth,
                                FIELD_WIDTH / 2.0 + distanceFromCenterY - width / 2.0,
                                height);
        }

        public static class BlueOutpost {
                private BlueOutpost() {
                        /* This utility class should not be instantiated */
                }

                public static final double openingWidth = Units.inchesToMeters(31.8);
                public static final double openingHeight = Units.inchesToMeters(7.0);
                public static final double openingZ = Units.inchesToMeters(28.1);

                public static final double tag29Y = Units.inchesToMeters(26.22);

                // Reference point
                public static final Translation2d center = new Translation2d(0.0, tag29Y);
        }

        public static Translation3d blueToRed(final Translation3d translation) {
                return new Translation3d(
                                Field.FIELD_LENGTH - translation.getX(),
                                translation.getY(),
                                translation.getZ());
        }

        public static Translation2d blueToRed(final Translation2d translation) {
                return new Translation2d(
                                Field.FIELD_LENGTH - translation.getX(),
                                translation.getY());
        }

        public static double blueToRed(final double translation) {
                return Field.FIELD_LENGTH - translation;
        }

        @Getter
        public static final Distance fuelRadius = Inches.of(5.91 / 2.0);

        @Getter
        public static final Translation3d blueHubCenter = BlueHub.topCenter; // new Translation3d(
        // Units.inchesToMeters(182.11),
        // Units.inchesToMeters(158.84),
        // Units.inchesToMeters(72));
        @Getter
        public static final Translation3d redHubCenter = blueToRed(BlueHub.topCenter); // new Translation3d(
        // Units.inchesToMeters(469.11),
        // Units.inchesToMeters(158.84),
        // Units.inchesToMeters(72));

        @Getter
        private static final double APRIL_TAG_WIDTH = Units.inchesToMeters(6.50);

        /** Returns {@code true} if the robot is on the blue alliance. */
        public static boolean isBlue() {
                return DriverStation.getAlliance()
                                .orElse(DriverStation.Alliance.Blue)
                                .equals(DriverStation.Alliance.Blue);
        }

        /** Returns {@code true} if the robot is on the red alliance. */
        public static boolean isRed() {
                return !isBlue();
        }

        public static final Trigger red = new Trigger(Field::isRed);
        public static final Trigger blue = new Trigger(Field::isBlue);
}