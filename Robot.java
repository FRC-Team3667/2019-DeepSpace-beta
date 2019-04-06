package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;

import com.analog.adis16448.frc.ADIS16448_IMU;

public class Robot extends TimedRobot {
  // Talons
  WPI_TalonSRX _frontTLeftMotor;
  WPI_TalonSRX _frontTRightMotor;
  WPI_TalonSRX _rearTRightMotor;
  WPI_TalonSRX _rearTLeftMotor;
  MecanumDrive _mDrive;
  Joystick _joy1;
  Joystick _joy2;
  private UsbCamera camera;

  // Network Tables
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;
  NetworkTable table;

  // Serial Port Information
  SerialPort theThePort;

  public enum RunningInMode {
    none, teleop, auton, test;
  }

  RunningInMode runMode = RunningInMode.none;

  // Line Tracker
  DigitalInput lineTracker0;
  DigitalInput lineTracker1;
  DigitalInput lineTracker2;
  DigitalInput lineTracker3;
  DigitalInput lineTracker4;

  // Lifter Logic
  double frontLiftSpeed = 0.50;
  double rearLiftSpeed = 0;
  double maxFrontLiftSpeed = 0.9;
  double maxRearLiftSpeed = 0.9;

  // 10 Degrees of Freedom
  ADIS16448_IMU imu;
  double zDegree = 0;
  double xDegree = 0;
  double yDegree = 0;
  boolean didItAlready = false;
  boolean imuIsWorkingCorrectly = true; // IMU is Working or Not

  // Line Tracker Values
  double pastZDegree = zDegree;
  int zDegreeIterations = 0;
  double targetDegree = 0;
  double rotationCounter = 1;
  double turnRotation = 0;
  double forwardMotion = 0;
  boolean lTrack0 = false;
  boolean lTrack1 = false;
  boolean lTrack2 = false;
  boolean lTrack3 = false;
  boolean lTrack4 = false;
  boolean autoTrackingEnabled = false;

  // Sections of code to include or exclude
  boolean nTables = false; // Network Tables in Use
  boolean cServer = false; // Camera Server
  boolean jCam = false; // Jevois Camera
  boolean lTrack = false; // Line Tracker
  boolean tenDegrees = false; // 10 degrees of freedom
  boolean pneumatics = false; // Pneumatics System
  boolean limitSwitches = false; // limit switches
  boolean joystick2Enabled = false;

  // Timer
  Timer robotTimer = new Timer();

  // Pneumatics
  Compressor scottCompressor;
  DoubleSolenoid pneuVacuum;
  DoubleSolenoid pneuHatchPanelTop;
  DoubleSolenoid pneuHatchPanelBottom;
  boolean pneuEnabled = false;
  boolean pneuVaccumeIsOn = false;
  double vaccumeEndTime = 0;

  // climbing vars
  double stopClimbTime = 0;
  double startClimbDegree = 0;
  boolean climbInitialize = true;

  // Limit switches
  DigitalInput limitSwitchIntakeLift;
  DigitalInput limitSwitchRearLift;

  double betaSpeedModifier = 0.80;

  @Override
  public void robotInit() {

    // Setup the joystick
    try {
      _joy1 = new Joystick(0);
      // _joy1 = new XboxController(0);
    } catch (Exception ex) {
    }
    // Setup the Drive System
    _frontTLeftMotor = new WPI_TalonSRX(13);
    _frontTRightMotor = new WPI_TalonSRX(12);
    _rearTRightMotor = new WPI_TalonSRX(11);
    _rearTLeftMotor = new WPI_TalonSRX(10);
    // Invert all the motors, they're probably wired wrong
    _frontTLeftMotor.setInverted(true);
    _frontTRightMotor.setInverted(true);
    _rearTLeftMotor.setInverted(true);
    _rearTRightMotor.setInverted(true);
    _frontTLeftMotor.setNeutralMode(NeutralMode.Brake);
    _frontTRightMotor.setNeutralMode(NeutralMode.Brake);
    _rearTLeftMotor.setNeutralMode(NeutralMode.Brake);
    _rearTRightMotor.setNeutralMode(NeutralMode.Brake);
    _mDrive = new MecanumDrive(_frontTLeftMotor, _rearTLeftMotor, _frontTRightMotor, _rearTRightMotor);

    // If our camera is onboard the roborio - This year it's not
    if (cServer) {
      try {
        camera = CameraServer.getInstance().startAutomaticCapture(0);
      } catch (Exception ex) {
      }
      if (camera == null) {
        camera = null; // This will prevent a "Warning" error during compilation
      }
    }

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    runMode = RunningInMode.auton;
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void teleopInit() {
    runMode = RunningInMode.teleop;
  }

  @Override
  public void teleopPeriodic() {
    forwardMotion = _joy1.getRawAxis(1) * -1 * betaSpeedModifier;
    double strafe = 0;
    if (_joy1.getRawAxis(0) > 0.05 || _joy1.getRawAxis(0) < -0.05) {
      strafe = _joy1.getRawAxis(0) * 1.25 * betaSpeedModifier;
      if (strafe > 1.0) {
        strafe = 1.0;
      }
    }
    turnRotation = _joy1.getRawAxis(4) * 0.4  * betaSpeedModifier;
    _mDrive.driveCartesian(strafe, forwardMotion, turnRotation, 0);
  }

  @Override
  public void testPeriodic() {
    teleopPeriodic();
    runMode = RunningInMode.test;
  }
}
