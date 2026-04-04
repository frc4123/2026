package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Sim.Mode;

public class ShiftHelpers {

    public static boolean blueWinAutoSimRandom = Math.random() > 0.5;

    public static boolean blueWonAuto() {
        if (Constants.Sim.CURRENT_MODE == Mode.SIM) {
            return ShiftHelpers.blueWinAutoSimRandom;
        }
        final String matchInfo = DriverStation.getGameSpecificMessage();
        if (matchInfo != null && matchInfo.length() > 0) {
            return matchInfo.charAt(0) == 'B';
        }
        // Safe default if data isn't ready yet
        return false;
    }

    public static int timeLeftInShiftSeconds(final double currentMatchTime) {
        if (currentMatchTime >= 130) {
            SmartDashboard.putBoolean("Won Auto?", ShiftHelpers.didWeWinAuto());
            return (int) (currentMatchTime - 130);
        } else if (currentMatchTime >= 105 && currentMatchTime <= 130) {
            return (int) (currentMatchTime - 105);
        } else if (currentMatchTime >= 80 && currentMatchTime <= 105) {
            return (int) (currentMatchTime - 80);
        } else if (currentMatchTime >= 55 && currentMatchTime <= 80) {
            return (int) (currentMatchTime - 55);
        } else if (currentMatchTime >= 30 && currentMatchTime <= 55) {
            return (int) (currentMatchTime - 30);
        } else {
            return (int) currentMatchTime;
        }
    }

    public static boolean isCurrentShiftBlue(final double currentMatchTime) {
        if (currentMatchTime >= 105 && currentMatchTime <= 130) {
            return ShiftHelpers.blueWonAuto() ? false : true;
        } else if (currentMatchTime >= 80 && currentMatchTime <= 105) {
            return ShiftHelpers.blueWonAuto() ? true : false;
        } else if (currentMatchTime >= 55 && currentMatchTime <= 80) {
            return ShiftHelpers.blueWonAuto() ? false : true;
        } else if (currentMatchTime >= 30 && currentMatchTime <= 55) {
            return ShiftHelpers.blueWonAuto() ? true : false;
        } else {
            return true;
        }
    }

    public static boolean isFiveSecBeforeShiftChange(final double currentMatchTime) {
        if (DriverStation.isAutonomous()) {
            return false;
        }
        return ShiftHelpers.timeLeftInShiftSeconds(currentMatchTime) == 5 ? true : false;
    }

    public static boolean isTwelveSecBeforeShiftChange(final double currentMatchTime) {
        if (DriverStation.isAutonomous()) {
            return false;
        }
        return ShiftHelpers.timeLeftInShiftSeconds(currentMatchTime) == 12 ? true : false;
    }

    public static boolean isTwoSecBeforeShiftChange(final double currentMatchTime) {
        if (DriverStation.isAutonomous()) {
            return false;
        }
        return ShiftHelpers.timeLeftInShiftSeconds(currentMatchTime) < 2 ? true : false;
    }

    public static boolean didWeWinAuto() {
        if ((ShiftHelpers.blueWonAuto() && Field.isBlue())
                || (!ShiftHelpers.blueWonAuto() && Field.isRed())) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean currentShiftIsYours() {
        final double currentMatchTime = DriverStation.getMatchTime();
        if (currentMatchTime > 130) {
            return true;
        }
        if (currentMatchTime <= 30) {
            return true;
        }
        final boolean isBlueShift = ShiftHelpers.isCurrentShiftBlue(currentMatchTime);
        if (Field.isBlue()) {
            return isBlueShift;
        } else {
            return !isBlueShift;
        }
    }
}
