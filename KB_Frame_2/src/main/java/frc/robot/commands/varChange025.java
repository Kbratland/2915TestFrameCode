package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class varChange025 extends CommandBase{
    
    
    @Override
    public void initialize(){
        RobotContainer.speedLimit = 0.25;
        System.out.println(RobotContainer.speedLimit);
    }
}
