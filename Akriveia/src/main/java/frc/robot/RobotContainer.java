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
import frc.robot.commands.auton.FollowGivenPath;
import frc.robot.commands.auton.HolonomicAuton;
import frc.robot.commands.auton.Slalom;
import frc.robot.commands.teleop.*;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.MasterConstants;
import frc.robot.subsystems.AngledShooter;
import frc.robot.subsystems.BallTransfer;
import frc.robot.subsystems.Drum;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PowerManagement;

import java.util.ArrayList;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class RobotContainer {
  public Intake m_Intake;
  private Controller m_Controller;
  private Compressor m_Compressor;
  private AngledShooter m_AngledShooter;
  private Drum m_Drum;
  private Drivetrain m_Drivetrain;
  private BallTransfer m_BallTransfer;
  private Flywheel m_Flywheel;
  private PowerManagement m_PowerManagement;

  private SendableChooser<String> m_autonChooser;
  private static final String m_bounce = "s_bounce";
  private static final String m_barrel_racing = "s_barrel_racing";
  private static final String m_galactic_search_a = "s_galactic_search_a";
  private static final String m_galactic_search_b = "s_galactic_search_b";
  private static final String m_slalom = "s_slalom";
  private static final String m_easy = "s_easy";
  private static final String m_forward = "s_forward";
  private static final String m_forward_and_back = "s_forward_and_back";
  private static final String m_l_shape = "s_l_shape";

  public Trajectory m_trajectory;

  private ArrayList<Trajectory> m_trajLibrary; 

  public ProfiledPIDController thetaController;

  public SwerveControllerCommand m_SwerveControllerCommand;

  public RobotContainer() {
    m_Compressor = new Compressor();
    m_Compressor.start();

    m_Controller = Controller.getInstance();

    m_Intake = Intake.getInstance();
    m_Drum = Drum.getInstance();
    m_BallTransfer = BallTransfer.getInstance();
    m_Flywheel = Flywheel.getInstance();
    m_AngledShooter = AngledShooter.getInstance();
    m_PowerManagement = PowerManagement.getInstance();

    m_PowerManagement.clearFaults();

    m_Drivetrain = Drivetrain.getInstance();
    m_Drivetrain.setDefaultCommand(new SwerveDrive(
      () -> m_Controller.getForward(), 
      () -> m_Controller.getStrafe(),
      () -> m_Controller.getSpin(),
      () -> m_Controller.isFieldCentricButtonPressed(),
      () -> m_Controller.isRobotCentricButtonPressed(),
      () -> m_Controller.isBackRobotCentricButtonPressed()));

    configureButtonBindings();

    m_autonChooser = new SendableChooser<>();
    m_autonChooser.addOption("Barrel Racing", m_barrel_racing);
    m_autonChooser.addOption("Bounce Path", m_bounce);
    m_autonChooser.setDefaultOption("Forward", m_forward);
    m_autonChooser.addOption("Forward and Back", m_forward_and_back);
    m_autonChooser.addOption("Easy", m_easy);
    m_autonChooser.addOption("Galactic Search A", m_galactic_search_a);
    m_autonChooser.addOption("Galactic Search B", m_galactic_search_b);
    m_autonChooser.addOption("L_Shape", m_l_shape);
    m_autonChooser.addOption("Slalom", m_slalom);
    SmartDashboard.putData("Auton Choice", m_autonChooser);


    // Please keep below array trajFilelist in alphabetical order.  Makes it easier to keep track.
    String[] trajFilelist = {"barrel_racing", "bounce", "easy", "forward_and_back", "forward",
      "galactic_search_pathA_blueBalls", "galactic_search_pathA_redBalls", 
      "galactic_search_pathB_blueBalls", "galactic_search_pathB_redBalls",
      "L_shape", "slalom"};
    m_trajLibrary = new ArrayList<Trajectory>();
    for (String var : trajFilelist)  {
      m_trajLibrary.add(preloadCreateTrajectory(var));
    }
    //preloadTrajectoryFiles();

    SmartDashboard.putData("ATLAS IntStellarAccuracy", new AtlasForInterstellarAccuracy());
    SmartDashboard.putData("ATLAS PowerPort", new AtlasForPowerPort());
    SmartDashboard.putData("ATLAS Global Off", new AtlasOff());
  }

  private void configureButtonBindings() {

    // Drivetrain buttons
    m_Controller.getDefenseButton().whenHeld(new SwerveDefense());
    m_Controller.getVisionButton().whenHeld(new SwerveVision( 
      () -> m_Controller.getForward(), 
      () -> m_Controller.getStrafe()));
    m_Controller.getResetGyroButton().whenHeld(new SwerveResetGyro());
    m_Controller.getResetGyroButton().whenPressed(new SwerveResetGyro());
    m_Controller.getLockSpinButton().whenPressed(new SwerveLockedSpin());
    m_Controller.getTurboButton().whenHeld(new SwerveTurboOn());
    m_Controller.getTurboButton().whenReleased(new SwerveTurboOff());
    m_Controller.getOrbitButton().whenHeld(new SwerveOrbit(
      () -> m_Controller.getOrbitSpeed(),
      () -> m_Controller.getPOVTransStick()));

    m_Controller.getIntakeSpinButton().whenActive(new IntakeRoll());
    m_Controller.getIntakeReverseButton().whenActive(new IntakeReverse());
  
    m_Controller.getDrumUpButton().whenActive(new DrumVelocitySpeed());
    m_Controller.getDrumDownButton().whenActive(new DrumVelocitySlow());
  
    m_Controller.getTransferStraightButton().whenActive(new BallTransferStraight());
    m_Controller.getTransferPivotButton().whenActive(new BallTransferPivotAndRoll());
  
    m_Controller.getIntakeHarvestButton().whenActive(new IntakeHarvest());
    m_Controller.getIntakeHarvestResetButton().whenActive(new IntakeHarvestReset());
  
    // m_Controller.getDrumAgitateButton().whenActive(new DrumAgitate());
    m_Controller.getDrumAgitatePreShootButton().whenActive(new DrumAgitatePreShoot());
  
    m_Controller.getDrumInputResetButton().whenActive(new DrumInputReset());
    m_Controller.getDrumCCWButton().whenActive(new DrumCCW());
  
    m_Controller.getPOVUp().whenHeld(new AngledShooterUp());
    m_Controller.getPOVDown().whenHeld(new AngledShooterDown());
    m_Controller.getPOVLeft().whenHeld(new FlywheelSlow());
    m_Controller.getPOVRight().whenHeld(new FlywheelSpeed());

    m_Controller.getAngledShooterOffButton().whenActive(new AngledShooterOff());
  
    m_Controller.getShootButton().whenActive(new Shoot());
    m_Controller.getResetShootButton().whenActive(new ShootReset());
    
  }

  public Command getAutonomousCommand() {

        /*TrajectoryConfig config =
        new TrajectoryConfig(
               4.468,
                1.631)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

    */

    double startTime = Timer.getFPGATimestamp();

    if (m_autonChooser.getSelected().equals("s_barrel_racing")) {
      // System.out.print("################ INFO: setting trajectory = barrel_racing ###########################");
      m_trajectory = m_trajLibrary.get(0);
      //createTrajectory("barrel_racing");
      //return new FollowGivenPath(trajectory);
    }
    else if(m_autonChooser.getSelected().equals("s_bounce")) {
      // System.out.print("################ INFO: setting trajectory = bounce ###########################");
      m_trajectory = m_trajLibrary.get(1);
      //createTrajectory("bounce");
      //return new FollowGivenPath(trajectory);
    }
    else if(m_autonChooser.getSelected().equals("s_easy")) {
      // System.out.print("################ INFO: setting trajectory = easy ###########################");
      m_trajectory = m_trajLibrary.get(2);
      //createTrajectory("easy");
      //return new FollowGivenPath(trajectory);
    } 
    else if(m_autonChooser.getSelected().equals("s_forward_and_back")) {
      // System.out.print("################ INFO: setting trajectory = forward_and_back ###########################");
      m_trajectory = m_trajLibrary.get(3);
      //createTrajectory("forward_and_back");
      //return new FollowGivenPath(trajectory);
    }
    else if(m_autonChooser.getSelected().equals("s_forward")) {
      // System.out.print("################ INFO: setting trajectory = forward ###########################");
      m_trajectory = m_trajLibrary.get(4);
      //createTrajectory("forward");
      //return new FollowGivenPath(trajectory);
    }
    else if (m_autonChooser.getSelected().equals("s_galactic_search_a")) {
      // System.out.print("################ INFO: setting trajectory = galactic_search_a ###########################");
      //createTrajectory("galactic_search.pathA_redBalls");
      //return new FollowGivenPath(trajectory);
    }
    else if (m_autonChooser.getSelected().equals("s_galactic_search_b")) {
      // System.out.print("################ INFO: setting trajectory = galactic_search_b ###########################");
      //createTrajectory("galactic_search>pathA_redBalls");
      //return new FollowGivenPath(trajectory);
    }
    else if(m_autonChooser.getSelected().equals("s_l_shape")) {
      // System.out.print("################ INFO: setting trajectory = l_shape ###########################");
      m_trajectory = m_trajLibrary.get(9);
      //createTrajectory("L_shape");
      //return new FollowGivenPath(trajectory);
    }
    else if(m_autonChooser.getSelected().equals("s_slalom")) {
      // System.out.print("################ INFO: setting trajectory = slalom ###########################");
      m_trajectory = m_trajLibrary.get(10);
      //createTrajectory("slalom");
      //new FollowGivenPath(trajectory);
    }
   
    // System.out.println("#####################################################");
    // System.out.println("################               WARNING      WARNING:               ############");
    // System.out.println("#####              BIG OLE CHUNK OF DATA     ########"); 
    // System.out.println("#####              ABOUT TO DUMPED HERE.     ########"); 
    // System.out.println("############     tractory object contains following: ");
    // System.out.println("###############  ");
    // System.out.println("###############   BEGIN TRAJECTORY DATA:  ");
    // System.out.println("");
    // System.out.println(m_trajectory.toString()); 
    // System.out.println("");
    // System.out.println("###############   END TRAJECTORY DATA:  ");
    // System.out.println("###############  ");
    // System.out.println("############        THAT'S ALL, FOLKS!!   ############"); 
    // System.out.println("######################################################");



    TrajectoryConfig config = 
      new TrajectoryConfig(
      4.48,
      1.631)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

    // An example trajectory to follow.  All units in meters.
    /*
    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config); 
      */
    
    Pose2d initialTrajPose = m_trajectory.getInitialPose();


    m_Drivetrain.resetOdometry(initialTrajPose);  // Reset odometry to the starting pose of the trajectory.

    /*
    m_SwerveControllerCommand =
      new SwerveControllerCommand(
        m_trajectory,
        m_Drivetrain::getCurrentPose, 
        DrivetrainConstants.DRIVE_KINEMATICS,

        // Position controllers
        new PIDController(DrivetrainConstants.P_X_Controller, 0, 0),
        new PIDController(DrivetrainConstants.P_Y_Controller, 0, 0),
        thetaController,
        m_Drivetrain::setModuleStates,
        m_Drivetrain);

    if(m_SwerveControllerCommand == null) { 
       System.out.println("#############                        Ya screwed up, bub.                                      #########"); 
       System.out.println("###########  ERROR: RobotContainer.getAutonoumousCommand() m_SwerveControllerCommand is null. #########"); 
    }

    return m_SwerveControllerCommand.andThen(() -> m_Drivetrain.drive(0, 0, 0)); 
    */

    return new HolonomicAuton(m_trajectory);
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
}
