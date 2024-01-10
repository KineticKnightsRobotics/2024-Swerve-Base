package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.LimeLight;
import frc.robot.lib.PID_Config;
import frc.robot.subsystems.SwerveDrive;

public class LimeLight_Strafe extends CommandBase {

    LimeLight   SUBSYSTEM_LIMELIGHT;
    SwerveDrive SUBSYSTEM_SWERVEDRIVE;
    PIDController strafePID;
    double /*xSpeed,ySpeed,*/currentX,currentY,targetX,targetY,deltaX,deltaY;
    double velocityX;
    DoubleSupplier ySpeed;
    double targetDistance = 2.5;


    public LimeLight_Strafe(LimeLight m_subsystem1, SwerveDrive m_subsystem2){
        addRequirements(m_subsystem1,m_subsystem2);
        SUBSYSTEM_LIMELIGHT  = m_subsystem1;
        SUBSYSTEM_SWERVEDRIVE= m_subsystem2;
    }
    @Override
    public void initialize(){
        strafePID = new PIDController(
        PID_Config.VisionDriving.Strafing.Proportional,
        PID_Config.VisionDriving.Strafing.Integral,
        PID_Config.VisionDriving.Strafing.Proportional
        );

    }
    @Override
    public void execute(){
        //get Limelight Data
        //0: X displacement
        //1: Y displacement
        //2: Z displacement

        // Limelight Z -> Robot Y

        double[] positionalData = SUBSYSTEM_LIMELIGHT.getBotPose_LimeLight();

        double deltaX = positionalData[0];
        double deltaY = positionalData[2];


        //double speedX = 0;
        //double speedY = 0;

        double speedX = strafePID.calculate(deltaX, 0);
        SmartDashboard.putNumber("speed X", speedX);
        
        double speedY = strafePID.calculate(deltaY, 1);
        SmartDashboard.putNumber("speed Y", speedY);

        ChassisSpeeds newChassisSpeeds = new ChassisSpeeds(speedX,speedY,0.0);

        SUBSYSTEM_SWERVEDRIVE.setChassisSpeed(newChassisSpeeds);



    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return false;
    }
    
}