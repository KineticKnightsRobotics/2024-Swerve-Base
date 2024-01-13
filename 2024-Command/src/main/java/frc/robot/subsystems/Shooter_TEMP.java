package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.lib.Constants.Shooter;
import frc.robot.lib.PID_Config.ShooterPID;


public class Shooter_TEMP extends SubsystemBase {

    CANSparkMax MOTOR_SHOOTER_RIGHT;
    CANSparkMax MOTOR_SHOOTER_LEFT;

    PIDController speedController;
    

    



    public Shooter_TEMP(){
        MOTOR_SHOOTER_RIGHT = new CANSparkMax(Shooter.ID_MOTOR_CONVEYER_RIGHT, MotorType.kBrushless);
        MOTOR_SHOOTER_LEFT = new CANSparkMax(Shooter.ID_MOTOR_CONVEYER_LEFT, MotorType.kBrushless);

       // MOTOR_CONVEYER_RIGHT.restoreFactoryDefaults();
       // MOTOR_CONVEYER_RIGHT.restoreFactoryDefaults();


        MOTOR_SHOOTER_RIGHT.setInverted(true);
        MOTOR_SHOOTER_LEFT.setInverted(false);
        
        MOTOR_SHOOTER_RIGHT.follow(MOTOR_SHOOTER_LEFT);

        speedController = new PIDController(ShooterPID.Proportional, ShooterPID.Integral, ShooterPID.Proportional);

    }

    public void setShooterPercentOutput(double percentOutput){
        MOTOR_SHOOTER_RIGHT.set(percentOutput);
        MOTOR_SHOOTER_LEFT.set(percentOutput);
        


        //MOTOR_SHOOTER_LEFT.set(speedController.calculate(MOTOR_SHOOTER_LEFT.getAppliedOutput(),percentOutput));

    }
    

}
