package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.lib.Constants.IntakeSubsystemConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Intake extends SubsystemBase {

    private final CANSparkMax roller = new CANSparkMax(IntakeSubsystemConstants.ID_INTAKE_ROLLER, MotorType.kBrushless);

    public Intake() {

        this.roller.restoreFactoryDefaults();
        this.roller.setClosedLoopRampRate(2.0);

    };

    public void setIntakeSpeed(double speed) {
        roller.set(speed);
    }

    public Command runIntake(double speed) {
        return Commands.runOnce(() -> setIntakeSpeed(speed));
    }



    
}
