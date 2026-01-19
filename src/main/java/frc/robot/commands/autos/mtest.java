package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class mtest extends SubsystemBase{
    public Command metertest(){
        return AutoBuilder.buildAuto("5meter test");
    }
}