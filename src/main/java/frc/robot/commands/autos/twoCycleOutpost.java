package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class twoCycleOutpost extends SubsystemBase{
    public Command twoCycleOutpostRight(){
        return AutoBuilder.buildAuto("2CycleOutpostRight");
    }
}