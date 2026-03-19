// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;

// import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.FuelSim;
import frc.robot.utils.ShiftHelpers;
import frc.robot.utils.ShotCache;

public class Robot extends LoggedRobot {

    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    private int dashboardCounter = 0;

    // /* log and replay timestamp and joystick data */
    // private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
    //     .withTimestampReplay()
    //     .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        if(Constants.Sim.CURRENT_MODE == Constants.Sim.Mode.Sim) {
            Logger.addDataReceiver(new NT4Publisher());
            Logger.start();
        }
        SmartDashboard.putString("Driver Checklist", 
            "-----Brady-----\n Plug in ethernet until clickMake sure controller inputs are in order\nSelect appropriate auto\n\n-----Milton + Joseph\n-----Is Battery percentage above 30%\n2 Pis are plugged firmly into external battery\nQuest is plugged firmly into external battery\nZero Intake\n Zero Hood\n" );
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotPeriodic() {
        //m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        ShotCache.update();

        // SmartDashboard.putNumber("Target Hood Angle", ShotCache.get().getHoodAngle().in(Degrees));
        // SmartDashboard.putNumber("Target Exit Velocity", ShotCache.get().getExitVelocity().in(MetersPerSecond));
        if (++dashboardCounter >= 50) {
            dashboardCounter = 0;

            SmartDashboard.putNumber("Match Data/MatchTime", DriverStation.getMatchTime());
            SmartDashboard.putBoolean("Match Data/InShift", ShiftHelpers.currentShiftIsYours());
            SmartDashboard.putNumber("Match Data/TimeLeftInShift",
            ShiftHelpers.timeLeftInShiftSeconds(DriverStation.getMatchTime()));
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
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
