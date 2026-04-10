// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Oculus;
import frc.robot.utils.FuelSim;
import frc.robot.utils.ShiftHelpers;
import frc.robot.utils.ShotCache;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {

    private Command autonomousCommand;
    private final RobotContainer robotContainer;

    private int dashboardCounter = 0;

    // /* log and replay timestamp and joystick data */
    // private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
    // .withTimestampReplay()
    // .withJoystickReplay();

    public Robot() {
        this.robotContainer = new RobotContainer();
        if (Constants.Sim.CURRENT_MODE == Constants.Sim.Mode.SIM) {
            Logger.addDataReceiver(new NT4Publisher());
            Logger.start();
        }
        SmartDashboard.putString(
                "Driver Checklist",
                ">>   Brady   <<\n 1. Plug in ethernet until click\n2. Make sure controller inputs are in order\n3. Select appropriate auto\n\n>>   Milton & Joseph   <<\n1. Is Battery percentage above 30%\n2. Two PI's are plugged firmly into external battery\n3. Quest is plugged firmly into external battery\n4. Zero Intake\n5.  Zero Hood\n");
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotPeriodic() {
        // m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        ShotCache.update();

        // SmartDashboard.putNumber("Target Hood Angle",
        // ShotCache.get().getHoodAngle().in(Degrees));
        // SmartDashboard.putNumber("Target Exit Velocity",
        // ShotCache.get().getExitVelocity().in(MetersPerSecond));
        if (++this.dashboardCounter >= 25) {
            this.dashboardCounter = 0;
            SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
            SmartDashboard.putNumber("Match Data/MatchTime", DriverStation.getMatchTime());
            SmartDashboard.putBoolean("Match Data/InShift", ShiftHelpers.currentShiftIsYours());
            SmartDashboard.putNumber(
                    "Match Data/TimeLeftInShift",
                    ShiftHelpers.timeLeftInShiftSeconds(LoggedRobot.defaultPeriodSecs));
            SmartDashboard.putBoolean("Trust Quest", Oculus.trustQuest());
            ShiftHelpers.timeLeftInShiftSeconds(DriverStation.getMatchTime());
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    // TODO init vision pose for localizers here?
    @Override
    public void autonomousInit() {
        this.autonomousCommand = this.robotContainer.getAutonomousCommand();

        if (this.autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(this.autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (this.autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(this.autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        FuelSim.getInstance().updateSim();
    }
}
