package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class varChange10 extends CommandBase{
    
    
    @Override
    public void initialize(){
        RobotContainer.speedLimit = 1.0;
    }
}
