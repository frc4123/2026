package frc.robot.utils;

public class ShootingStatus {

    private static boolean isShooting = false;

    public static boolean isShooting() {
        return isShooting;
    }

    public static void setIsShooting(boolean flag) {
        isShooting = flag;
    }
}