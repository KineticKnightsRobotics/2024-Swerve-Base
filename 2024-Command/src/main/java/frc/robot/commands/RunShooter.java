package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter_TEMP;


public class RunShooter extends Command {
    
    double percentOutput;

    Shooter_TEMP m_subsystem1;

    public RunShooter(Shooter_TEMP subsystem){
        addRequirements(subsystem);
        m_subsystem1 = subsystem;
    }
    @Override
    public void execute() {
        m_subsystem1.setShooterPercentOutput(0.50);
    }
    @Override
    public void end(boolean interupted) {
        m_subsystem1.setShooterPercentOutput(0.0);
    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
