package frc.robot.lib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;

public class LimeLight extends SubsystemBase {


    NetworkTable LIMELIGHT = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry LIMELIGHT_TX = LIMELIGHT.getEntry("tx");
    NetworkTableEntry LIMELIGHT_TY = LIMELIGHT.getEntry("ty");
    NetworkTableEntry LIMELIGHT_TA = LIMELIGHT.getEntry("ta");
    NetworkTableEntry LIMELIGHT_TV = LIMELIGHT.getEntry("tv");

    NetworkTableEntry LIMELIGHT_ROBOT_POSE = LIMELIGHT.getEntry("targetpose_cameraspace");

    NetworkTableEntry LIMELIGHT_APRILTAG_TRANSLATION = LIMELIGHT.getEntry("targetpose_robotspace");
    //NetworkTableEntry LIMELIGHT_APRILTAG_DISTANCE = 

    public PIDController strafePID;

    public LimeLight() {
        strafePID = new PIDController(
            PID_Config.VisionDriving.Strafing.Proportional,
            PID_Config.VisionDriving.Strafing.Integral,
            PID_Config.VisionDriving.Strafing.Derivitive
        );
        //SmartDashboard.putData(strafePID);
        setName("Test PID");
        
       SendableRegistry.setName(strafePID, "PID Drive");

        ShuffleboardTab tab = Shuffleboard.getTab("LiveWindow");
GenericEntry drivePIDTestEnable = tab.add("DrivePID Enable", false).getEntry();

    }
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("LimeLight Has Target",getLimeLightTV());
        SmartDashboard.putNumber("LimeLight X Delta", getLimeLightTX());
        SmartDashboard.putNumber("LimeLight Y Delta", getLimeLightTY());
        SmartDashboard.putNumber("LimeLight Target Area", getLimeLightTA());

        SmartDashboard.putNumberArray("Apriltag Data", getBotPose_LimeLight());

        SmartDashboard.putNumber("April Tag Detected", LIMELIGHT.getEntry("tid").getDouble(0.0));


    }

    public double getLimeLightTX() {
        return LIMELIGHT_TX.getDouble(0.0);
    }
    public double getLimeLightTY() {
        return LIMELIGHT_TY.getDouble(0.0);
    }
    public double getLimeLightTA() {
        return LIMELIGHT_TA.getDouble(0.0);
    }
    public boolean getLimeLightTV() {
        return LIMELIGHT_TV.getDouble(0.0) == 1;
    }
    public double[] getBotPose_LimeLight(){
        return LIMELIGHT_ROBOT_POSE.getDoubleArray(new double[0]);
    }
    public void setPipeline(double pipelineID){
        LIMELIGHT.getEntry("pipeline").setNumber(pipelineID);
    }
    public Command changePipeline(double pipelineID) {
        return Commands.runOnce(()->setPipeline(pipelineID));
    }

}
