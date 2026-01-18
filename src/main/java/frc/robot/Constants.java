package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class CanIdCanivore { 

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

        public static final int Elevator = 15;

        public static final int Algae_Arm = 17;
    }

    public static final class InputConstants {
        public static final int kDriverControllerPort0 = 0;
        public static final int kDriverControllerPort1 = 1;
        public static final int kDriverControllerPort2 = 2;
        public static final boolean fieldOrientation = true;
        public static final double kDeadband = 0.028;
    }

    public static final class Quest {

        //Front Forward Camera Translation and Angle
        public static final double frontX = Units.inchesToMeters(8); // 7.495 7.176364 -7.176364
        public static final double frontY = Units.inchesToMeters(-4.5);
        public static final double frontZ = Units.inchesToMeters(12); // 7.02

        public static final double frontRoll = Math.toRadians(0);
        public static final double frontPitch = Math.toRadians(0); // negative pitch is up according to 25 code
        public static final double frontYaw = Math.toRadians(0);
        //TODO: get precise cords

        public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);
        // Trust down to 2cm in X directio
        // Trust down to 2cm in Y direction 0.035
        // Trust down to 2 degrees rotational);
    }   

    public static final class VisionConstants {
        //Front Forward Camera Translation and Angle
        public static final double frontX = Units.inchesToMeters(-11.5); // 7.495 7.176364 -7.176364
        public static final double frontY = Units.inchesToMeters(0);
        public static final double frontZ = Units.inchesToMeters(8.186116); // 7.02

        public static final Pose3d blueHub = new Pose3d(4.625, 4.035, 1.829, new Rotation3d());
        public static final Pose3d redHub = new Pose3d(11.920, 4.035, 1.829, new Rotation3d());
        //TODO: DOUBLE CHECK THE HEIGHT OF HUB AND THE CORDINATE POSITIONS ARE GUESSES

        public static final double frontRoll = Math.toRadians(0);
        public static final double frontPitch = Math.toRadians(-60); // negative pitch is up according to 25 code
        public static final double frontYaw = Math.toRadians(20);
    }
}
