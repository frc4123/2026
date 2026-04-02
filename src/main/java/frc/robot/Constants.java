package frc.robot;

import java.util.Set;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANdi;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    private Constants() {
        /* This utility class should not be instantiated */
    }

    public static final class CanIdCanivore {
        private CanIdCanivore() {
            /* This utility class should not be instantiated */
        }

        public static final CANBus canivore = new CANBus("Good Boy CANivore 10");

        public static final int Front_Left_Drive = 2;
        public static final int Front_Right_Drive = 3;
        public static final int Back_Left_Drive = 4;
        public static final int Back_Right_Drive = 5;
        // drive motors - order - start top left in clockwise rotation

        public static final int Front_Left_Turn = 6;
        public static final int Front_Right_Turn = 7;
        public static final int Back_Left_Turn = 8;
        public static final int Back_Right_Turn = 9;
        // turn motors - order - start top left in clockwise rotation

        public static final int Pigeon = 10;

        public static final int Front_Left_CANcoder = 11;
        public static final int Front_Right_CANcoder = 12;
        public static final int Back_Left_CANcoder = 13;
        public static final int Back_Right_CANcoder = 14;

        public static final int Turret = 15;
        public static final int Turret_Encoder1 = 16;
        public static final int Turret_Encoder2 = 17;

        public static final int Intake_CANdi = 18;

        public static final int Intake_Arm = 19;

        public static final int Intake_Roller = 20;

        public static final int Hood = 21;

        public static final int Shooter = 23;

        public static final int Climb = 24;

        public static final int SevenEleven = 25;

        public static final int Uptake = 26;
    }

    public static final class InputConstants {
        private InputConstants() {
            /* This utility class should not be instantiated */
        }

        public static final int kDriverControllerPort0 = 0;
        public static final int kDriverControllerPort1 = 1;
        public static final int kDriverControllerPort2 = 2;
        public static final boolean fieldOrientation = true;
        public static final double kDeadband = 0.028;
    }

    public static final class SwerveConstants {
        private SwerveConstants() {
            /* This utility class should not be instantiated */
        }

        public static final double CLOSE_TRANSLATION_PP_KP = 2.5; // 8
        public static final double CLOSE_TRANSLATION_PP_KI = 0.0;
        public static final double CLOSE_TRANSLATION_PP_KD = 0.0;

        public static final double CLOSE_ROTATION_PP_KP = 5; // 8
        public static final double CLOSE_ROTATION_PP_KI = 0.0;
        public static final double CLOSE_ROTATION_PP_KD = 0.0;

        public static final double driverDeadband = 0.05;

        public static final double shootOnTheMoveError = 1.1; // .675, 1.75

        public static final Pose2d BLUE_CLIMB_POSE = new Pose2d(0.0080772, 3.7457125999999996,
                new Rotation2d(0 * Math.PI / 180.0)); // id 31
        public static final Pose2d RED_CLIMB_POSE = new Pose2d(16.53296166, 4.3235626,
                new Rotation2d(0 * Math.PI / 180.0)); // id 15

        public static final double[][] ADDITIONS = {
                { 1.0549228, 1.0422874 }, // LEFT ADDITION // {0.342, 0} //0.385
                { 1.0549228, -1.0297126 } // RIGHT ADDITION // {0.342, 0.348} //0.385 was correct in odometry w
                                          // advantagescope
                // driver relative -> {+forward/back-, +left/right-}
        };

        public static final Pose2d robotToClimber = new Pose2d(Units.inchesToMeters(2),
                Units.inchesToMeters(27.5 / 2.0), new Rotation2d());
        // 33.75 bumper width
    }

    public static class IntakeArmConstants {
        private IntakeArmConstants() {
            /* This utility class should not be instantiated */
        }

        public static final CANdi intakeCANdi = new CANdi(
                Constants.CanIdCanivore.Intake_CANdi,
                Constants.CanIdCanivore.canivore);

        static {
            IntakeArmConstants.intakeCANdi.optimizeBusUtilization();
        }

        public static final double outPosition = 0;
        public static final double stowPosition = 0.33;
        public static final double midPosition = IntakeArmConstants.stowPosition / 1.7;

        public static final double sensorToMechanismRatio = 25.0;
        public static final double currentCancelationThreshold = 80;

        public static final double kP = 1000.0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 2.5;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kG = 6.14123;

        public static final double velocity = 2.5;
        public static final double slowVelocity = 0.11;
        public static final double midVelocity = 0.33;
        public static final double acceleration = 3.5;
    }

    public static class IntakeRollerConstants {
        private IntakeRollerConstants() {
            /* This utility class should not be instantiated */
        }

        public static final Distance STOW_POS = edu.wpi.first.units.Units.Inches.of(0);
        public static final Distance DEPLOY_POS = edu.wpi.first.units.Units.Inches.of(10.875);
        public static final Voltage SPIN_VOLTAGE = edu.wpi.first.units.Units.Volts.of(3);

        public static final double VEL_MULTIPLIER = 70.0; // multiplies goal velocity for targetting
        public static final double VEL_POWER = 0.3; // raises goal velocity to power
        public static final LinearVelocity BASE_VEL = edu.wpi.first.units.Units.InchesPerSecond.of(50); // added to
                                                                                                        // final
                                                                                                        // velocity

        public static final double zeroVelo = 0;
        public static final double intakeVelo = 60; // was 60, 31
        public static final double reverseVelo = -20;
        public static final double intakeAcc = 120; // was 90

        public static final double kP = 0.426;
        public static final double kI = 0; // was 0.0441
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0.118; // try this 0;
        public static final double kA = 0;
    }

    public static class SevenElevenConstants {
        private SevenElevenConstants() {
            /* This utility class should not be instantiated */
        }

        public static final double zeroVelo = 0;
        public static final double sevenElevenLowVelo = 25; // 9
        public static final double sevenElevenMidVelo = 37; // 12
        public static final double sevenElevenHighVelo = 60; // 90, 5
        public static final double sevenElevenReverseVelo = -20;

        public static final double sevenElevenAcc = 60;// 25 //7, //30

        public static final double kP = 0.426;
        public static final double kI = 0; // was 0.0441
        public static final double kD = 0;
        public static final double kS = 0.05;
        public static final double kV = 0.118; // try this 0;
        public static final double kA = 0;
    }

    public static class UptakeConstants {
        private UptakeConstants() {
            /* This utility class should not be instantiated */
        }

        public static final double zeroVelo = 0;
        public static final double uptakeVelo = 60; // 40 was 55, but change to 45 if bad
        public static final double reverseVelo = -45;
        public static final double uptakeAcc = UptakeConstants.uptakeVelo * 5.0;

        public static final double kP = 0.426;
        public static final double kI = 0; // was 0.0441
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0.118; // try this 0;
        public static final double kA = 0;
    }

    public static final class TurretConstants {
        private TurretConstants() {
            /* This utility class should not be instantiated */
        }

        public static final double stowPosition = 0;
        public static final double velocity = 4; // 3
        public static final double acceleration = 8; // 6

        public static final double dragCoeff = 1.14123;

        public static final double kP = 20; // 20 //either p is too low
        public static final double kI = 1.5; // or I is too high
        public static final double kD = 20; // or D is too high? lower d and increase i first thing tmr
        public static final double kS = 3;// 3
        public static final double kV = 5.5;// 6; //5.75 //check if turret velo is below what it is set in velocity 4 to
                                            // see if kv is too low
        public static final double kA = 1;// 1.4123;

        // IN ROTATIONS //
        public static final double mechanismMinRange = -37.0 / 72.0; // -1 is -360 degrees
        public static final double mechanismMaxRange = 37.0 / 72.0; // 1 is +360 degrees
        // this makes total of 720 degrees rotation^^^^

        public static final double mechanismGearTeeth = 85.0; // drives botrh encoders
        public static final double encoder1Teeth = 10.0; // 24.0; // Gear on Hex Shaft A that connects to turret
        public static final double encoder2Teeth = 22.0; // 10.0 / (50.0/22.0); // 22.0;
        public static final double turretDrivingGear = 10.0;
        public static final double fiftyTGear = 50.0;

        public static final double motorToTurretRatio = (TurretConstants.fiftyTGear / TurretConstants.turretDrivingGear)
                * (TurretConstants.mechanismGearTeeth / TurretConstants.encoder1Teeth);// (48.0/9.0) * (180.0/24.0);

        public static final double rotorToEncoder1Ratio = (TurretConstants.fiftyTGear
                / TurretConstants.turretDrivingGear); // 48.0 / 9.0;
        public static final double sensorToMechanismRatio = (TurretConstants.mechanismGearTeeth
                / TurretConstants.encoder1Teeth); // 180.0 / 24.0;
        public static final double sensor2ToMechanismRatio = ((TurretConstants.mechanismGearTeeth
                / TurretConstants.encoder1Teeth)
                * (TurretConstants.fiftyTGear / TurretConstants.encoder2Teeth));

        // public static final double encoder2Ratio = (turretGearTeeth / encoder1Teeth)
        // * (50.0 / encoder2Teeth); // Gear on Hex Shaft B that connects to turret

        public static final double encoder1Offset = 0; // -0.575684;
        public static final double encoder2Offset = 0;

        public static final double encoder1CRTOffset = -0.09668; // -0.575684;
        public static final double encoder2CRTOffset = -0.165283; // -0.481281;
        // TODO: if the wrap happens to be near the zero measurement (within hundredths
        // check yams for interval confirmation), then RESEAT CANCODERS

        public static final double coverageMargin = 1.2;
        public static final int minTeeth = 1;
        public static final int maxTeeth = 1;
        public static final int maxIterations = 30;

        public static final double offsetX = Units.inchesToMeters(7);
        public static final double offsetY = Units.inchesToMeters(0);
        public static final double offsetZ = Units.inchesToMeters(21);

        public static final double tagOffset = Units.inchesToMeters(47 / 2);
        public static final double[] validTurretTagsBlue = { 21, 26, 18 };
        public static final double[] validTurretTagsRed = { 2, 10, 5 };

        public static final Translation2d turretOffset = new Translation2d(TurretConstants.offsetX,
                TurretConstants.offsetY);
        public static final Pose3d robotToTurret = new Pose3d(TurretConstants.offsetX, TurretConstants.offsetY,
                TurretConstants.offsetZ, new Rotation3d());
        public static final Transform3d transform3D = new Transform3d(TurretConstants.robotToTurret, new Pose3d());

        public static final Transform2d robotToTurretTransform = new Transform2d(
                TurretConstants.robotToTurret.getTranslation().toTranslation2d(),
                TurretConstants.robotToTurret.getRotation().toRotation2d());

        public static final Distance DISTANCE_ABOVE_FUNNEL = edu.wpi.first.units.Units.Inches.of(6); // was 20
    }

    public static final class ShooterConstants {
        private ShooterConstants() {
            /* This utility class should not be instantiated */
        }

        public static final LinearVelocity MIN_SPEED = edu.wpi.first.units.Units.MetersPerSecond.of(6);

        public static final Distance flywheelRadius = edu.wpi.first.units.Units.Inches.of(2.03);

        public static final double metersPerRotation = 2.0 * Math.PI
                * ShooterConstants.flywheelRadius.in(edu.wpi.first.units.Units.Meters);

        public static final Distance compression = edu.wpi.first.units.Units.Inches.of(0.25);

        public static final double sensorTomechanfismGearTeeth = 1.0 / 1.0;

        public static final double shootingTestErrorRatio = 1.2355 + 0.25824123; // 0.2624123, 0.2675 0.272, 0.235,
                                                                                 // 0.245, 0.23, 0.21 0.18, 0.15, 0.21
                                                                                 // 0.225 0.275, 0.2, 0.15

        public static final double kP = 12;
        public static final double kI = 0;
        public static final double kD = 0.0035; // 0.0025
        public static final double kS = 4.00025;
        public static final double kV = 0.114123;
        public static final double kA = 0;

        public static final double acceleration = 1300;
        public static final double slowAcceleration = 9;
    }

    public static final class HoodConstants {
        private HoodConstants() {
            /* This utility class should not be instantiated */
        }

        public static final Angle MIN_HOOD_ANGLE = Degrees.of(29.359724); // its really 29.359724
        public static final Angle MAX_HOOD_ANGLE = Degrees.of(71.359724);

        public static final Angle MAX_HOOD_SHOT_ANGLE = edu.wpi.first.units.Units.Degrees.of(60); // 65

        public static final double sensorToMechanismRatio = 136.0 / 1.0;

        public static final double kP = 13;
        public static final double kI = 0.051;
        public static final double kD = 0.6;
        public static final double kS = 2.1;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double slowVelocity = 40;
        public static final double velocity = 140;
        public static final double acceleration = 300;

        public static final double stowPosition = HoodConstants.MAX_HOOD_ANGLE.in(edu.wpi.first.units.Units.Degrees);
    }

    public static final class ClimbConstants {
        private ClimbConstants() {
            /* This utility class should not be instantiated */
        }

        public static final double downPosition = 0;
        public static final double upPosition = 50;
        public static final double testPosition = ClimbConstants.upPosition * 0.1;

        public static final double kP = 65;
        public static final double kI = 0;
        public static final double kD = 1.5;
        public static final double kS = 2;
        public static final double kV = 0.5;
        public static final double kA = 0;
        public static final double kG = 0;

        public static final double velocity = 100;
        public static final double acceleration = 200;
    }

    public static final class Quest {
        private Quest() {
            /* This utility class should not be instantiated */
        }

        // Front Forward Camera Translation and Angle
        public static final double frontX = Units.inchesToMeters(12.5);// 12.5 // 7.495 7.176364 -7.176364 //12.75
        public static final double frontY = Units.inchesToMeters(-9.5); // -9 //-9.498
        public static final double frontZ = Units.inchesToMeters(10); // 10// 7.02 //10

        public static final double frontRoll = Math.toRadians(0);
        public static final double frontPitch = Math.toRadians(0); // negative pitch is up according to 25 code
        public static final double frontYaw = Math.toRadians(0);

        public static Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(1.6, 1.6, 3.2);
        public static final double questUpdate = 10.00; // 30
    }

    public static final class VisionConstants {
        private VisionConstants() {
            /* This utility class should not be instantiated */
        }

        public static final double ambiguityThreshold = 0.06;// 0.06

        // FLO = Front_Left_Outside camera
        public static final double FLO_frontX = Units.inchesToMeters(9.069); // 7.495 7.176364 -7.176364
        public static final double FLO_frontY = Units.inchesToMeters(11.097);
        public static final double FLO_frontZ = Units.inchesToMeters(7.826); // 7.02

        public static final double FLO_frontRoll = Math.toRadians(0);
        public static final double FLO_frontPitch = Math.toRadians(-15.0); // -75.0 negative pitch is up according to 25
                                                                           // code
        public static final double FLO_frontYaw = Math.toRadians(132.273);

        // FLI = Front_Left_Inside camera
        public static final double FLI_frontX = Units.inchesToMeters(10.836); // 7.495 7.176364 -7.176364
        public static final double FLI_frontY = Units.inchesToMeters(9.396);
        public static final double FLI_frontZ = Units.inchesToMeters(7.979); // 7.02

        public static final double FLI_frontRoll = Math.toRadians(0);
        public static final double FLI_frontPitch = Math.toRadians(-30.0); // -60 negative pitch is up according to 25
                                                                           // code
        public static final double FLI_frontYaw = Math.toRadians(30);

        // FRI = Front_Right_Inside camera
        public static final double FRI_frontX = Units.inchesToMeters(10.229); // 7.495 7.176364 -7.176364
        public static final double FRI_frontY = Units.inchesToMeters(-10.140);
        public static final double FRI_frontZ = Units.inchesToMeters(7.825761); // 7.02

        public static final double FRI_frontRoll = Math.toRadians(0);
        public static final double FRI_frontPitch = Math.toRadians(-15.0); // -75.0 negative pitch is up according to 25
                                                                           // code
        public static final double FRI_frontYaw = Math.toRadians(-131.987);

        // FRO = Front_Right_Outside camera
        public static final double FRO_frontX = Units.inchesToMeters(9.411); // 7.495 7.176364 -7.176364
        public static final double FRO_frontY = Units.inchesToMeters(-10.998);
        public static final double FRO_frontZ = Units.inchesToMeters(13.929); // 7.02

        public static final double FRO_frontRoll = Math.toRadians(0);
        public static final double FRO_frontPitch = Math.toRadians(-15); // -75.0 negative pitch is up according to 25
                                                                         // code
        public static final double FRO_frontYaw = Math.toRadians(0);

        // blueHub translations
        public static final Pose3d blueHub = new Pose3d(4.625, 4.035, 1.4304264/* 1.828 */, new Rotation3d());
        public static final Translation2d blueHubTranslation2d = VisionConstants.blueHub.getTranslation()
                .toTranslation2d();
        public static final Translation3d blueHubTranslation3d = VisionConstants.blueHub.getTranslation();

        // blueDepot pose
        public static final Pose3d blueDepot = new Pose3d(-1.5, 6.0, 0.2, new Rotation3d());

        // blueLeftBumpCorner pose
        // public static final Pose3d blueLeftBumpCorner = new Pose3d (3.5, 7, 0.2, new
        // Rotation3d()); //x4.03 and y was +1

        // blueLeftBumpCorner pose
        // public static final Pose3d blueRightBumpCorner = new Pose3d (3.5, 1, 0.2, new
        // Rotation3d()); //x4.03 and y was -1

        // redAimThreshold pose
        public static final Pose3d blueAimThreshold = new Pose3d(-1.5, 2.080, 0.2, new Rotation3d());

        // redHub translations
        public static final Pose3d redHub = new Pose3d(11.920, 4.035, 1.4304264/* 1.828 */, new Rotation3d());
        public static final Translation2d redHubTranslation2d = VisionConstants.redHub.getTranslation()
                .toTranslation2d();
        public static final Translation3d redHubTranslation3d = VisionConstants.redHub.getTranslation();

        // redAimThreshold pose
        public static final Pose3d redAimThreshold = new Pose3d(18.5, 6.0, 0.2, new Rotation3d());

        // redRightBumpCorner pose
        public static final Pose3d redRightBumpCorner = new Pose3d(13.5, 7, 0.2, new Rotation3d()); // 12.505 and y was
                                                                                                    // + 1

        // redRightBumpCorner pose
        public static final Pose3d redLeftBumpCorner = new Pose3d(13.5, 1, 0.2, new Rotation3d()); // 12.505 and y was
                                                                                                   // -1

        // blueDepot pose
        public static final Pose3d redDepot = new Pose3d(18.5, 2.080, 0.2, new Rotation3d());

        // threshold for how close we are to the blue bump/trench for auto rotation
        public static final double blueLeftBumpOrTrenchThreshold = 2.5;
        public static final double blueRightBumpOrTrenchThreshold = 7.25;

        // threshold for how close we are to the bump/trench for auto rotation
        public static final double redLeftBumpOrTrenchThreshold = 10.0;
        public static final double redRightBumpOrTrenchThreshold = 14.0;

        public static final double topBumpTrenchEdge = 6.627;
        public static final double bottomBumpTrenchEdge = 1.43;

        // Max acceptable roll and pitch to recieve photon data
        public static final double MAX_ACCEPTABLE_PITCH = 5;
        public static final double MAX_ACCEPTABLE_ROLL = 5;
    }

    public static final class Sim {

        public enum RobotMode {
            COMP,
            PRACTICE
        }

        public static final RobotMode DS_MODE = RobotMode.COMP;

        public enum Mode {
            REAL, SIM, REPLAY
        }

        public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

        public static final double fullWidth = Units.inchesToMeters(27);
        public static final double fullLength = Units.inchesToMeters(27);
        public static final double fullHeight = Units.inchesToMeters(22);
    }

    public static final class FieldConstants {
        private FieldConstants() {
            /* This utility class should not be instantiated */
        }

        public static final int[] BLUE_TAGS = { 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32 };
        public static final int[] RED_TAGS = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };

        public static final Set<Integer> RED_HUB_TAG_IDS = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
        public static final Set<Integer> BLUE_HUB_TAG_IDS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

        public static final Distance FIELD_LENGTH = edu.wpi.first.units.Units.Inches.of(650.12);
        public static final Distance FIELD_WIDTH = edu.wpi.first.units.Units.Inches.of(316.64);

        public static final Distance ALLIANCE_ZONE = edu.wpi.first.units.Units.Inches.of(156.06);

        // public static final Translation3d HUB_BLUE =
        // new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
        // public static final Translation3d HUB_RED =
        // new Translation3d(FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2),
        // Inches.of(56.4));
        public static final Distance FUNNEL_RADIUS = edu.wpi.first.units.Units.Inches.of(24);
        public static final Distance FUNNEL_HEIGHT = edu.wpi.first.units.Units.Inches.of(72 - 56.4);
    }
}
