package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ConstantAimToggleCmd;
import frc.robot.commands.FastAutoAimCmd;
import frc.robot.commands.IntakeNoteCmd;
import frc.robot.commands.OverrideCmd;
import frc.robot.commands.ResetHeadingCmd;
import frc.robot.commands.SetArmPitchCmd;
import frc.robot.commands.SwerveRotateToAngle;
import frc.robot.commands.ShootNoteCmd;
import frc.robot.commands.SmallPnuematicsCmd;
import frc.robot.commands.DefaultCommands.IntakeMotorCmd;
import frc.robot.commands.DefaultCommands.PitchMotorCmd;
import frc.robot.commands.DefaultCommands.PneumaticsCmd;
import frc.robot.commands.DefaultCommands.ShooterMotorsCmd;
import frc.robot.commands.DefaultCommands.SwerveJoystickCmd;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.PitchMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.ShootingMotorSubsystem;

public class RobotContainer {
  private final String SPtwoNtwoNone = "3 notes in speaker Speaker 2 Note 2 - 1 - TESTED", SPtwoNtwo = "2 notes in speaker Seaker 2 Note 2 - TESTED",
  SPtwoNoneNtwoNthree = "4 notes in speaker from Speaker 2 Notes 1 - 2 - 3", SPtwoNthreeNtwoNoneNfour = "4 in speaker Speaker 2 Notes 3 - 2 - 1 - 4",
  SPtwoNoneNfour = "3 in speaker Speaker 2 Notes 2 - 1 pick up 4", SPtwoNtwoNfour = "3 in speaker Speaker 2 Notes 2 - 4",
  SPoneNoneNfourRSPone = "3 in speaker Speaker one Notes 1 - 4", SPthreeNthreeNeightNseven = "3 in speaker Speaker 3 Notes 3 - 8 - 7",
  SPthreeNfourNfive = "3 in speaker Speaker 3 Notes 4 - 5", SPthreeNfiveNfour = "3 in speaker Speaker 3 Notes 5 - 4",
  SPthreeNthree = "2 in speaker Speaker 3 Note 3", SPoneNone = "2 in speaker Speaker 1 Note 1", 
  SPthreeNeightNseven = "3 in speaker Speaker 3 Notes 8 - 7", SpThreeNThreeNEight = "3 in speaker out of the way stage side", test = "test",
  justShoot = "just shoot";
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final CommandSequences commandSequences = new CommandSequences();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();


  private final IntakeMotorSubsystem intakeMotorSubsystem = new IntakeMotorSubsystem();
  private final PitchMotorSubsystem pitchMotorSubsystem = new PitchMotorSubsystem();
  private final ShootingMotorSubsystem shootingMotorSubsystem = new ShootingMotorSubsystem();

  //private final Limelights limelights = new Limelights(swerveSubsystem, armSubsystem);
  XboxController xbox = new XboxController(OperatorConstants.kXboxControllerPort);
  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoyPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoyPort);
  
  Field2d m_field = new Field2d();
  ChoreoTrajectory traj;

  public RobotContainer() {
    traj = Choreo.getTrajectory("Trajectory");

    m_field.getObject("traj").setPoses(
      traj.getInitialPose(), traj.getFinalPose()
    );
    m_field.getObject("trajPoses").setPoses(
      traj.getPoses()
    );

    SmartDashboard.putData(m_field);

    pitchMotorSubsystem.setDefaultCommand(new PitchMotorCmd(pitchMotorSubsystem, () -> xbox.getLeftY(), () -> leftJoystick.getRawButton(1), () -> swerveSubsystem.getPose())); // Intake Motors
    intakeMotorSubsystem.setDefaultCommand(new IntakeMotorCmd(intakeMotorSubsystem, () -> leftJoystick.getRawButton(1),
    () -> xbox.getYButton()));
    shootingMotorSubsystem.setDefaultCommand(new ShooterMotorsCmd(shootingMotorSubsystem, () -> xbox.getYButton(), () -> swerveSubsystem.getPose()));

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
      ()-> leftJoystick.getY(),
      ()-> leftJoystick.getX(),
      ()-> -rightJoystick.getX(),
      ()-> rightJoystick.getRawButton(1)
    ));

    pneumaticsSubsystem.setDefaultCommand(new PneumaticsCmd(pneumaticsSubsystem));
    
    configureBindings();


    m_chooser.addOption(SPtwoNtwoNone, SPtwoNtwoNone);
    m_chooser.addOption(SPtwoNtwo, SPtwoNtwo);
    m_chooser.addOption(SPtwoNoneNtwoNthree, SPtwoNoneNtwoNthree);
    m_chooser.addOption(SPtwoNthreeNtwoNoneNfour, SPtwoNthreeNtwoNoneNfour);
    m_chooser.addOption(SPtwoNoneNfour, SPtwoNoneNfour);
    m_chooser.addOption(SPtwoNtwoNfour, SPtwoNtwoNfour);
    m_chooser.addOption(SPoneNoneNfourRSPone, SPoneNoneNfourRSPone);
    m_chooser.addOption(SPthreeNthreeNeightNseven, SPthreeNthreeNeightNseven);
    m_chooser.addOption(SPthreeNfourNfive, SPthreeNfourNfive);
    m_chooser.addOption(SPthreeNfiveNfour, SPthreeNfiveNfour);
    m_chooser.addOption(SPthreeNthree, SPthreeNthree);
    m_chooser.addOption(SPoneNone, SPoneNone);
    m_chooser.addOption(SPthreeNeightNseven, SPthreeNeightNseven);
    m_chooser.addOption(SpThreeNThreeNEight, SpThreeNThreeNEight);
    m_chooser.addOption(test, test);
    m_chooser.addOption(justShoot, justShoot);
    

    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    driverBoard.add("Auto choices", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.addCamera("Limelight Stream Intake", "limelight_intake", "mjpg:http://limelight-intake.local:5800").withSize(4,4);
    driverBoard.addCamera("Limelight Stream Shooter", "limelight_shooter", "mjpg:http://limelight-shooter.local:5800").withSize(4,4);

    //warning a name change will break auto paths because pathplanner will not update it
    NamedCommands.registerCommand("runIntake", new IntakeNoteCmd(intakeMotorSubsystem, pitchMotorSubsystem, 0, 8));
    NamedCommands.registerCommand("Shoot", new ShootNoteCmd(shootingMotorSubsystem, intakeMotorSubsystem, .9 ));
    NamedCommands.registerCommand("autoShoot", new FastAutoAimCmd(pitchMotorSubsystem, swerveSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));
    NamedCommands.registerCommand("angle to speaker", new SetArmPitchCmd(pitchMotorSubsystem, Constants.ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle));
    NamedCommands.registerCommand("rotate to 270", new SwerveRotateToAngle(swerveSubsystem, CommandSequences.teamChangeAngle((270))));
    NamedCommands.registerCommand("SetArm to intake", new SetArmPitchCmd(pitchMotorSubsystem, Constants.ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle));
    NamedCommands.registerCommand("reset heading", new ResetHeadingCmd(swerveSubsystem, 0));
    NamedCommands.registerCommand("Set Heading 60", new ResetHeadingCmd(swerveSubsystem, 60));
    NamedCommands.registerCommand("Set Heading -60", new ResetHeadingCmd(swerveSubsystem, -60));

    AutoBuilder.configureHolonomic(
            () -> swerveSubsystem.getPose(), // Robot pose supplier for auto (correct range -180-180)
            swerveSubsystem ::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            () -> swerveSubsystem.getChassisSpeedsRobotRelative(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            swerveSubsystem :: runModulesRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
            () -> SwerveSubsystem.isOnRed(),
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            swerveSubsystem // Reference to this subsystem to set requirements
        );
        System.out.println("team " + SwerveSubsystem.isOnRed());
  }

  private void configureBindings() {
    new JoystickButton(rightJoystick, 4).onTrue(new InstantCommand(swerveSubsystem :: zeroHeading));
    new JoystickButton(rightJoystick, 3).onTrue(new OverrideCmd(swerveSubsystem, intakeMotorSubsystem, pitchMotorSubsystem, shootingMotorSubsystem));
    new JoystickButton(rightJoystick, 13).onTrue(new InstantCommand(swerveSubsystem :: disableCams));
    new JoystickButton(rightJoystick, 2).onTrue(new ShootNoteCmd(shootingMotorSubsystem, intakeMotorSubsystem, 0.9, 2000));

    new CommandXboxController(OperatorConstants.kXboxControllerPort).leftBumper().onTrue(new SmallPnuematicsCmd(pneumaticsSubsystem));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).rightBumper().onTrue(new InstantCommand(pneumaticsSubsystem :: toggleBigpneumatics));

    new CommandXboxController(OperatorConstants.kXboxControllerPort).a().onTrue(new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).b().onTrue(new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).x().onTrue(new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorAmpPresetAngle));

    new CommandXboxController(OperatorConstants.kXboxControllerPort).rightTrigger(0.5).onTrue(new ShootNoteCmd(shootingMotorSubsystem, intakeMotorSubsystem, 0.9, 4000));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).leftTrigger(0.5).onTrue(new ShootNoteCmd(shootingMotorSubsystem, intakeMotorSubsystem, 0.4, 0));
    //new CommandXboxController(OperatorConstants.kXboxControllerPort).y().onTrue(new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorStandbyPresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).start().onTrue(new FastAutoAimCmd(pitchMotorSubsystem, swerveSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).back().onTrue(new ConstantAimToggleCmd(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem));
  }

  public Command getAutonomousCommand() {
    System.out.println("Autos Begun");
       
    var thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    swerveSubsystem.resetOdometry(traj.getInitialPose());

    Command swerveCommand = Choreo.choreoSwerveCommand(
        traj, // Choreo trajectory from above
        swerveSubsystem::getPose, // A function that returns the current field-relative pose of the robot: your
                               // wheel or vision odometry
        new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // PIDController for field-relative X
                                                                                   // translation (input: X error in meters,
                                                                                   // output: m/s).
        new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), // PIDController for field-relative Y
                                                                                   // translation (input: Y error in meters,
                                                                                   // output: m/s).
        thetaController, // PID constants to correct for rotation
                         // error
        (ChassisSpeeds speeds) -> m_robotDrive.drive( // needs to be robot-relative
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            false),
        true, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
        swerveSubsystem // The subsystem(s) to require, typically your drive subsystem only
    );

    return Commands.sequence(
        Commands.runOnce(() -> swerveSubsystem.resetOdometry(traj.getInitialPose())),
        swerveCommand,
        swerveSubsystem.run(() -> swerveSubsystem.drive(0, 0, 0, false))
    );
  }

    return null;
  }

  public void logSwerve() {

    SmartDashboard.putNumber("Heading", swerveSubsystem.getHeading());
    SmartDashboard.putString("Robot Location", swerveSubsystem.getPose().getTranslation().toString());
    SmartDashboard.putNumber("frontLeft Encoder", swerveSubsystem.getFLAbsEncoder());
    SmartDashboard.putNumber("frontRight Encoder", swerveSubsystem.getFRAbsEncoder());
    SmartDashboard.putNumber("BackLeft Encoder", swerveSubsystem.getBLAbsEncoder());
    SmartDashboard.putNumber("BackRight Encoder", swerveSubsystem.getBRAbsEncoder());
    SmartDashboard.putNumber("Shooter speed", shootingMotorSubsystem.getShooterSpeed());
    SmartDashboard.putNumber("Rotation", swerveSubsystem.getRotation2d().getDegrees());
    //SmartDashboard.putNumber("Arm Encoder", armSubsystem.getAbsoluteEncoder());
  }

}
