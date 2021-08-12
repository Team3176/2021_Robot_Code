package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.auton.*;
import frc.robot.commands.teleop.*;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.MasterConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Drum;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.PowerManagement;

import java.util.ArrayList;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class RobotContainer {
  private Controller m_Controller;
  private Compressor m_Compressor;
  private Hood m_AngledShooter;
  private Drum m_Drum;
  private Drivetrain m_Drivetrain;
  private Transfer m_BallTransfer;
  private Flywheel m_Flywheel;
  private PowerManagement m_PowerManagement;

  private SendableChooser<String> m_autonChooser;
  private static final String m_autoDis5Back= "s_autoDis5Back";
  private static final String m_autoDis6Left= "s_autoDis6Left";
  private static final String m_autoDis6Right= "s_autoDis6Right";
  private static final String m_auto1Hypo = "s_auto1Hypo";
  private static final String m_auto2Hypo = "s_auto2Hypo";
  private static final String m_auto1Out = "s_auto1Out";
  private static final String m_auto2Out = "s_auto2Out";
  private static final String m_auto3Hypo = "s_auto3AIAEHypo";
  private static final String m_auto3Out = "s_auto3AIAEOut";
  private static final String m_autoCenterAT = "s_autoCenterAT";
  private static final String m_autonmutable1= "s_autonmutable1";
  private static final String m_autonmutable2= "s_autonmutable2";
  private static final String m_autonmutable3= "s_autonmutable3";

  public Trajectory m_trajectory;

  private ArrayList<Trajectory> m_trajLibrary; 

  public ProfiledPIDController thetaController;

  public SwerveControllerCommand m_SwerveControllerCommand;

  public RobotContainer() {
    m_Compressor = new Compressor();
    m_Compressor.start();

    m_Controller = Controller.getInstance();

    m_Drum = Drum.getInstance();
    m_Drum.setDefaultCommand(new DrumCCW());

    m_BallTransfer = Transfer.getInstance();
    m_Flywheel = Flywheel.getInstance();

    m_AngledShooter = Hood.getInstance();
    m_AngledShooter.setDefaultCommand(new HoodPosUp());

    m_PowerManagement = PowerManagement.getInstance();

    m_PowerManagement.clearFaults();

    m_Drivetrain = Drivetrain.getInstance();
    m_Drivetrain.setDefaultCommand(new SwerveDrive(
      () -> m_Controller.getForward(), 
      () -> m_Controller.getStrafe(),
      () -> m_Controller.getSpin()
      //() -> m_Controller.isFieldCentricButtonPressed(),
      //() -> m_Controller.isRobotCentricButtonPressed()
    ));

    configureButtonBindings();

    m_autonChooser = new SendableChooser<>();
    m_autonChooser.setDefaultOption("Auto: Dis5Back", m_autoDis5Back);
    m_autonChooser.addOption("Auto: Dis6Left", m_autoDis6Left);
    m_autonChooser.addOption("Auto: Dis6Right", m_autoDis6Right);
    m_autonChooser.addOption("Auto: 1Hypo", m_auto1Hypo);
    m_autonChooser.addOption("Auto: 2Hypo", m_auto2Hypo);
    m_autonChooser.addOption("Auto: 1Out", m_auto1Out);
    m_autonChooser.addOption("Auto: 2Out", m_auto2Out);
    m_autonChooser.addOption("Auto: 3Hypo", m_auto3Hypo);
    m_autonChooser.addOption("Auto: 3Out", m_auto3Out);
    m_autonChooser.addOption("Auto: Center Trench", m_autoCenterAT);
    m_autonChooser.addOption("Auton Mutable 1", m_autonmutable1);
    m_autonChooser.addOption("Auton Mutable 2", m_autonmutable2);
    m_autonChooser.addOption("Auton Mutable 3", m_autonmutable3);
    SmartDashboard.putData("Auton Choice", m_autonChooser);


    // Please keep below array trajFilelist in alphabetical order.  Makes it easier to keep track.
    /*
    String[] trajFilelist = {"barrel_racing", "bounce", "easy", "forward_and_back", "forward",
      "galactic_search_pathA_blueBalls", "galactic_search_pathA_redBalls", 
      "galactic_search_pathB_blueBalls", "galactic_search_pathB_redBalls",
      "L_shape", "slalom"};
    m_trajLibrary = new ArrayList<Trajectory>();
    for (String var : trajFilelist)  {
      m_trajLibrary.add(preloadCreateTrajectory(var));
    }
    */
    //preloadTrajectoryFiles();

    //SmartDashboard.putData("ATLAS IntStellarAccuracy", new AtlasForInterstellarAccuracy());
    //SmartDashboard.putData("ATLAS PowerPort", new AtlasForPowerPort());
    //SmartDashboard.putData("ATLAS Global Off", new AtlasOff());
  }

  private void configureButtonBindings() {

    // Drivetrain buttons
    m_Controller.getTransStick_Button1().whenHeld(new SwerveTurboOn());
    m_Controller.getTransStick_Button1().whenReleased(new SwerveTurboOff());
    m_Controller.getTransStick_Button2().whenHeld(new SwerveDefense());
    //m_Controller.getTransStick_Button3().whenHeld(new SwerveVision( 
    //  () -> m_Controller.getForward(), 
    //  () -> m_Controller.getStrafe()));
    m_Controller.getTransStick_Button3().whenPressed(new VisionToggleLeds());
    m_Controller.getTransStick_Button4().whenPressed(new SwitchCoordType());
    //m_Controller.getRotStick_Button1().whenHeld(new SwerveOrbit(
      //() -> m_Controller.getOrbitSpeed(),
      //() -> m_Controller.getPOVTransStick()));
    m_Controller.getRotStick_Button1().whenHeld(new AlignVizYawPLoopTele());
    m_Controller.getRotStick_Button4().whenPressed(new SwerveResetGyro());
    m_Controller.getRotStick_Button8().whenHeld(new SwerveResetGyro());
    m_Controller.getRotStick_Button8().whenPressed(new SwerveResetGyro());
    m_Controller.getRotStick_Button9().whenPressed(new SwerveLockedSpin());

  
    // m_Controller.getDrumUpButton().whenActive(new DrumVelocitySpeed());
    // m_Controller.getDrumDownButton().whenActive(new DrumVelocitySlow());

    /* ---- New Mapping ---- */

    // m_Controller.getOp_BumperRight().whenHeld(new ShootVision());
    // m_Controller.getOp_BumperRight().whenReleased(new ShootReset());
    // m_Controller.getOp_BumperRight().whenHeld(new HoodPosUp());
    // m_Controller.getOp_BumperRight().whenReleased(new HoodPosDown());
    m_Controller.getOp_BumperRight().whenActive(new ShootVisionSetUp());
    m_Controller.getOp_TriggerLeft().whenActive(new Shoot());
    m_Controller.getOp_TriggerRight().whenActive(new ShootReset());

    m_Controller.getOp_ButtonX().whenHeld(new TransferDown());
    m_Controller.getOp_ButtonX().whenReleased(new TransferUp());

    /* ---- End Mapping ---- */
    
    m_Controller.getOp_ButtonY().whenActive(new IntakeHarvestStart());
    m_Controller.getOp_ButtonYPlusBumperLeft().whenActive(new IntakeHarvestStop());
  
    // m_Controller.getOp_ButtonX().whenActive(new TransferDown());
    // m_Controller.getOp_ButtonXPlusBumperLeft().whenActive(new TransferUp());
  
    m_Controller.getOp_ButtonA().whenActive(new TransferDown());
    m_Controller.getOp_ButtonAPlusBumperLeft().whenActive(new TransferUp());

    m_Controller.getOp_ButtonB().whenActive(new HoodPosUp());
    m_Controller.getOp_ButtonBPlusBumperLeft().whenActive(new HoodPosDown());

    m_Controller.getOp_ButtonStart().whenActive(new HoodStop());
    m_Controller.getOp_ButtonStartPlusBumperLeft().whenActive(new DrumAgitatePreShoot());
  
    m_Controller.getOp_ButtonBack().whenActive(new DrumStop());
    m_Controller.getOp_ButtonBackPlusBumperLeft().whenActive(new DrumCCW());
  
    m_Controller.getOp_DpadUp().whenPressed(new HoodPosUp());    //ANGLE SHOOTER TEST
    m_Controller.getOp_DpadDown().whenPressed(new HoodPosDown());//ANGLE SHOOTER TEST
    m_Controller.getOp_DpadLeft().whenPressed(new FlywheelVelocityDown());
    m_Controller.getOp_DpadRight().whenPressed(new FlywheelVelocityUp());
  
    // m_Controller.getOp_BumperRight().whenActive(new Shoot());
    // m_Controller.getOp_BumperRightPlusBumperLeft().whenActive(new ShootVision());
    
    // m_Controller.getOp_TriggerRight().whenActive(new ShootReset());
    //m_Controller.getOp_TriggerLeft().whenActive(new ShootReset());

  }


  public Command getAutonomousCommand() {
    if (m_autonChooser.getSelected().equals("s_autoDis5Back")) {
      return new autoDis5Back();
    }
    else if(m_autonChooser.getSelected().equals("s_autonstatic2")) {
      return new autoDis6Left();
    }
    else if(m_autonChooser.getSelected().equals("s_autonstatic3")) {
      return new autoCenterAT();
    } 
    
    else if(m_autonChooser.getSelected().equals("s_autoDis5Back")) {return new autoDis5Back();}
    else if(m_autonChooser.getSelected().equals("s_autoDis6Left")) {return new autoDis6Left();}
    else if(m_autonChooser.getSelected().equals("s_autoDis6Right")) {return new autoDis6Right();}
    else if(m_autonChooser.getSelected().equals("s_auto1Hypo")) {return new auto1Hypo();}
    else if(m_autonChooser.getSelected().equals("s_auto2Hypo")) {return new auto2Hypo();}
    else if(m_autonChooser.getSelected().equals("s_auto1Out")) {return new auto1Out();}
    else if(m_autonChooser.getSelected().equals("s_auto2Out")) {return new auto2Out();}
    else if(m_autonChooser.getSelected().equals("s_auto3AIAEHypo")) {return new auto3AIAEHypo();}
    else if(m_autonChooser.getSelected().equals("s_auto3AIAEOut")) {return new auto3AIAEOut();}
    else if(m_autonChooser.getSelected().equals("s_autoCenterAT")) {return new autoCenterAT();}

    else if(m_autonChooser.getSelected().equals("s_autonmutable1")) {return new RunAutonMutable1();}
    else if(m_autonChooser.getSelected().equals("s_autonmutable2")) {return new RunAutonMutable2();}
    else if (m_autonChooser.getSelected().equals("s_autonmutable3")) {return new RunAutonMutable3();}
    
    return new autoDis5Back();
  }

  public void createTrajectory(String path){
    String trajectoryJSON = "paths/" + path + ".wpilib.json";
    //trajectory = null;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }

  public Trajectory preloadCreateTrajectory(String path){
    String trajectoryJSON = "paths/" + path + ".wpilib.json";
    Trajectory tempTrajectory = null;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      tempTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    return tempTrajectory;
  }

  public void stopAllMotors() {
    m_AngledShooter.stopMotors();
    m_BallTransfer.stopMotors();
    m_Drum.stopMotors();
    m_Flywheel.stopMotors();
    m_Drivetrain.stopMotors();
  }

  public void stopAllMotorsExceptDrivetrain() {
    m_AngledShooter.stopMotors();
    m_BallTransfer.stopMotors();
    m_Drum.stopMotors();
    m_Flywheel.stopMotors();
  }

}
