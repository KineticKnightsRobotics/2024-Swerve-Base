package frc.robot.subsystems;
//kauai
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
//wpi
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.Constants;
import frc.robot.lib.SwerveModule;
import frc.robot.lib.Constants.KinematicsConstants;
import frc.robot.lib.Constants.AutonomousConstants;
import frc.robot.lib.PID_Config;

import java.util.List;
//import java.util.Timer;
import java.util.concurrent.TimeUnit;


public class SwerveDrive extends SubsystemBase {
    
    private final SwerveModule MODULE_FRONT_LEFT = new SwerveModule(
        Constants.SwerveSubsystemConstants.ID_FRONT_LEFT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_FRONT_LEFT_MOTOR_DRIVE,
        Constants.SwerveSubsystemConstants.ID_FRONT_LEFT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_FRONT_LEFT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_FRONT_LEFT_ENCODER_ABSOLUTE
        ); 
    private final SwerveModule MODULE_BACK_LEFT = new SwerveModule(
        Constants.SwerveSubsystemConstants.ID_BACK_LEFT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_BACK_LEFT_MOTOR_DRIVE,
        Constants.SwerveSubsystemConstants.ID_BACK_LEFT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_BACK_LEFT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_BACK_LEFT_ENCODER_ABSOLUTE
        );
    private final SwerveModule MODULE_FRONT_RIGHT = new SwerveModule(
        Constants.SwerveSubsystemConstants.ID_FRONT_RIGHT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_FRONT_RIGHT_MOTOR_DRIVE,
        Constants.SwerveSubsystemConstants.ID_FRONT_RIGHT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_FRONT_RIGHT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_FRONT_RIGHT_ENCODER_ABSOLUTE
        );  
    private final SwerveModule MODULE_BACK_RIGHT = new SwerveModule(
        Constants.SwerveSubsystemConstants.ID_BACK_RIGHT_DRIVE,
        Constants.SwerveSubsystemConstants.REVERSED_BACK_RIGHT_MOTOR_DRIVE,
        Constants.SwerveSubsystemConstants.ID_BACK_RIGHT_TURN,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_TURN,
        Constants.SwerveSubsystemConstants.ID_BACK_RIGHT_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.REVERSED_ENCODER_ABSOLUTE,
        Constants.SwerveSubsystemConstants.OFFSET_BACK_RIGHT_ENCODER_ABSOLUTE
        );  

    private final AHRS navX = new AHRS();

    private final SwerveDriveOdometry ODEMETER = new SwerveDriveOdometry(KinematicsConstants.KINEMATICS_DRIVE_CHASSIS, getRotation2d(), getModulePositions());

    private final SwerveDrivePoseEstimator POSITION_ESTIMATOR = new SwerveDrivePoseEstimator(
        Constants.KinematicsConstants.KINEMATICS_DRIVE_CHASSIS,
        getRotation2d(),
        getModulePositions(),
        new Pose2d()
        );

    public SwerveDrive() {
        try {TimeUnit.SECONDS.sleep(1);}
        catch(InterruptedException e){}
        MODULE_FRONT_LEFT.resetEncoders();
        MODULE_FRONT_RIGHT.resetEncoders();
        MODULE_BACK_LEFT.resetEncoders();
        MODULE_BACK_RIGHT.resetEncoders();

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getRobotHeading());
        //SmartDashboard.putNumber("/timer", Timer.getMatchTime());
        SmartDashboard.putNumber("FL ABS ENC", MODULE_FRONT_LEFT.getAbsoluteEncoder());
        SmartDashboard.putNumber("FR ABS ENC", MODULE_FRONT_RIGHT.getAbsoluteEncoder());
        SmartDashboard.putNumber("BL ABS ENC", MODULE_BACK_LEFT.getAbsoluteEncoder());
        SmartDashboard.putNumber("BR ABS ENC",Math.toDegrees(MODULE_BACK_RIGHT.getAbsoluteEncoder()));

        MODULE_FRONT_LEFT.moduleData2Dashboard();
        MODULE_FRONT_RIGHT.moduleData2Dashboard();
        MODULE_BACK_LEFT.moduleData2Dashboard();
        MODULE_BACK_RIGHT.moduleData2Dashboard();

        POSITION_ESTIMATOR.update(getRotation2d(), getModulePositions());

        ODEMETER.update(
            getRotation2d(),
            getModulePositions()
        );
    }

    public double getRobotHeading() {
        return Math.IEEEremainder(Constants.SwerveSubsystemConstants.REVERSED_GYRO ? -navX.getAngle() : navX.getAngle() , 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getRobotHeading());
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        positions[0] = MODULE_FRONT_LEFT.getModulePosition();
        positions[1] = MODULE_FRONT_RIGHT.getModulePosition();
        positions[2] = MODULE_BACK_LEFT.getModulePosition();
        positions[3] = MODULE_BACK_RIGHT.getModulePosition();
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] newModuleStates = {
        MODULE_FRONT_LEFT.getModuleState(),
        MODULE_FRONT_RIGHT.getModuleState(),
        MODULE_BACK_LEFT.getModuleState(),
        MODULE_BACK_RIGHT.getModuleState()
        };
        return newModuleStates;
    }

    public void setChassisSpeed(ChassisSpeeds speed) {
        SwerveModuleState states[] = Constants.KinematicsConstants.KINEMATICS_DRIVE_CHASSIS.toSwerveModuleStates(speed);
        setModuleStates(states);
    }

    public void setModuleStates(SwerveModuleState states[]) {
        MODULE_FRONT_LEFT.setDesiredState(states[0]);
        MODULE_FRONT_RIGHT.setDesiredState(states[1]);
        MODULE_BACK_LEFT.setDesiredState(states[2]);
        MODULE_BACK_RIGHT.setDesiredState(states[3]);
    }

    public void zeroModules(){
        MODULE_FRONT_LEFT.resetEncoders();
        MODULE_FRONT_RIGHT.resetEncoders();
        MODULE_BACK_LEFT.resetEncoders();
        MODULE_BACK_RIGHT.resetEncoders();   
    }

    public SwerveControllerCommand startTrajectory() {
        //Config Trajectory Settings

        TrajectoryConfig config = new TrajectoryConfig(
            AutonomousConstants.LIMIT_AUTOSPEED_DRIVE,
            AutonomousConstants.LIMIT_AUTOSPEED_ROTATE
        );
        config.setKinematics(KinematicsConstants.KINEMATICS_DRIVE_CHASSIS);

        //Create trajectory

        Trajectory newTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(0.0, 0.5),
                new Translation2d(0.5, 0.0)
            ),
            new Pose2d(0,0,new Rotation2d(Math.PI)),
            config);

        //Create Controllers
        PIDController strafePID = new PIDController(
            PID_Config.VisionDriving.Strafing.Proportional,
            PID_Config.VisionDriving.Strafing.Integral,
            PID_Config.VisionDriving.Strafing.Derivitive
        );
        ProfiledPIDController rotationPID = new ProfiledPIDController(
            PID_Config.VisionDriving.Steering.Proportional,
            PID_Config.VisionDriving.Steering.Integral,
            PID_Config.VisionDriving.Steering.Derivitive,
            new TrapezoidProfile.Constraints(0.5 * Math.PI, 0.1 * Math.PI)
        );
        rotationPID.enableContinuousInput(0,360);

        return new SwerveControllerCommand(
            newTrajectory, 
            ()->ODEMETER.getPoseMeters(), 
            KinematicsConstants.KINEMATICS_DRIVE_CHASSIS,
            strafePID,
            strafePID,
            rotationPID,
            null,
            null,
         null);
    }


    public Command zeroRobotHeading() {
        return Commands.runOnce(() -> navX.zeroYaw());
    }

    public Command zeroModuleAngles() {
        return Commands.runOnce(()-> zeroModules());
    }

    public Command zeroRoboOdemetry() {
        return Commands.runOnce(()-> zeroModules());
    }

}
