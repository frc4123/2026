package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
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
        public static final Transform3d ROBOT_TO_QUEST = new Transform3d(0,0,0, null);
        //TODO: get precise cords

        public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);
        // Trust down to 2cm in X directio
        // Trust down to 2cm in Y direction 0.035
        // Trust down to 2 degrees rotational);
    }   

    public static final class VisionConstants {
        //Front Forward Camera Translation and Angle
        public static final double frontX = Units.inchesToMeters(9.911500); // 7.495 7.176364 -7.176364
        public static final double frontY = Units.inchesToMeters(13.492853); // -7.176364 7.495000 -7.495000
        public static final double frontZ = Units.inchesToMeters(8.186116); // 7.02

        public static final double frontRoll = Math.toRadians(0);
        public static final double frontPitch = Math.toRadians(-20); // 25
        public static final double frontYaw = Math.toRadians(20);
    }
}
