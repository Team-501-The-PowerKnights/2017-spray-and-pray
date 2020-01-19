
package com.powerknights.frc2017.robot;


import org.slf4j.Logger;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;
import com.powerknights.robot.riolog.Level;
import com.powerknights.robot.riolog.RioLogger;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;


/**
 * This is a demo program showing the use of the RobotDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right spinTime as
 * controlled by the switches on the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
public class Team501Robot
   extends SampleRobot
{

   /** Our classes' logger **/
   private static final Logger logger =
      RioLogger.getLogger( Team501Robot.class.getName() );

   static
   {
      // LOGGER - Override of default level
      // RioLogger.setLevel( logger, Level.TRACE );
      RioLogger.setLevel( logger, Level.DEBUG );
   }

   private final Preferences prefs;

   private final String cameraTableName = "/Camera";
   private final NetworkTable cameraTable;

   /*
	 * @formatter:off
	 *
	 * 'Forward' is Shooter End
	 * 'Backwards' is Gear End
	 * 'Left' is relative to Shooter End 'Forward'
	 * 'Right' is relative to Shooter End 'Forward'
	 *
	 * @formatter:on
	 */

   private PowerDistributionPanel pdp;

   private Joystick driverStick;
   private Joystick operatorStick;

   private VictorSP frontLeftMotor;
   private VictorSP backLeftMotor;
   private VictorSP frontRightMotor;
   private VictorSP backRightMotor;

   private RobotDrive drive;

   private Servo leftGearFlapServo;
   private Servo rightGearFlapServo;

   private CANTalon leftHangerMotor;
   private CANTalon rightHangerMotor;

   private CANTalon leftShooterMotor;
   private CANTalon rightShooterMotor;

   private CANTalon rightCANEncoder; // same as rightShooterMotor
   private CANTalon leftCANEncoder; // same as leftShooterMotor

   private AHRS ahrs;

   private AnalogInput gearDistanceSensor;

   private AnalogInput boilerDistanceSensor;

   private DigitalInput gearPegSensor;

   private SendableChooser< String > autoChooser;
   private SendableChooser< Boolean > autoGearPlaceChooser;
   private SendableChooser< Boolean > autoShootChooser;
   private SendableChooser< Boolean > visionSaveChooser;
   private SendableChooser< Boolean > useVisionChooser;
   private SendableChooser< Boolean > useGearSwitchChooser;
   private CameraUpdateListener vision;


   /**
   *
   **/
   public Team501Robot()
   {

      logger.info( "constructing" );

      logger.info( "BattleCry Competition Team501Robot()" );
      SmartDashboard.putString( "codeVersion", CodeVersionInfo.version );
      logger.info( "codeVersion={}", CodeVersionInfo.version );

      // Wait until we get the configuration data from driver station
      waitForDriverStationData();

      logger.info( "alliance={}, location={}",
         DriverStation.getInstance().getAlliance(),
         DriverStation.getInstance().getLocation() );

      // Pull the handles from the static predefined stuff
      prefs = Preferences.getInstance();
      cameraTable = NetworkTable.getTable( cameraTableName );

      logger.info( "constructed" );
   }


   /**
    * Holds the constructor until we receive at least one update of the control
    * data, which holds the run-spinTime configuration.
    **/
   private void waitForDriverStationData()
   {
      long count = 0;
      while ( !DriverStation.getInstance().isNewControlData() )
      {
         if ( ( count % 100 ) == 0 )
         {
            logger.trace( "Waiting ..." );
         }
         sleep( 100 );
         count++;
      }
   }


   /*
    * (non-Javadoc)
    *
    * @see edu.wpi.first.wpilibj.SampleRobot#robotInit()
    */
   @Override
   public void robotInit()
   {
      logger.info( "entering robotInit()" );

      pdp = new PowerDistributionPanel();

      driverStick = new Joystick( 0 );
      operatorStick = new Joystick( 1 );

      frontLeftMotor = new VictorSP( 0 );
      backLeftMotor = new VictorSP( 1 );
      frontRightMotor = new VictorSP( 2 );
      backRightMotor = new VictorSP( 3 );

      drive = new RobotDrive( frontLeftMotor, backLeftMotor, frontRightMotor,
         backRightMotor );
      drive.setSafetyEnabled( false );

      leftGearFlapServo = new Servo( 6 );
      rightGearFlapServo = new Servo( 7 );
      closeGearFlaps();

      leftHangerMotor = new CANTalon( 14 );
      leftHangerMotor.enableBrakeMode( true );
      rightHangerMotor = new CANTalon( 13 );
      rightHangerMotor.enableBrakeMode( true );
      configureHangerMotors( leftHangerMotor, rightHangerMotor );

      leftShooterMotor = new CANTalon( 15 );
      leftCANEncoder = leftShooterMotor;
      rightShooterMotor = new CANTalon( 12 ); // Encoder
      rightCANEncoder = rightShooterMotor;
      // configureShooterMotors( leftShooterMotor, rightShooterMotor );

      configureChassisEncoders( leftCANEncoder, rightCANEncoder );

      try
      {
         ahrs = new AHRS( SPI.Port.kMXP );
      }
      catch ( final RuntimeException ex )
      {
         logger.error( "Failed to instantiate navX MXP: " + ex.getMessage() );
      }
      ahrs.zeroYaw();

      gearDistanceSensor = new AnalogInput( 0 );
      gearDistanceSensor.setOversampleBits( 4 );
      gearDistanceSensor.setAverageBits( 2 );

      boilerDistanceSensor = new AnalogInput( 1 );
      boilerDistanceSensor.setOversampleBits( 4 );
      boilerDistanceSensor.setAverageBits( 2 );

      gearPegSensor = new DigitalInput( 5 );

      autoChooser = new SendableChooser<>();
      autoGearPlaceChooser = new SendableChooser<>();
      autoShootChooser = new SendableChooser<>();
      visionSaveChooser = new SendableChooser<>();
      useVisionChooser = new SendableChooser<>();
      useGearSwitchChooser = new SendableChooser<>();
      initChoosers();

      initCameraNetworkTables();
      // new Thread( new VisionAngleDashboardUpdater() ).start();

      resetDistanceDriven();
      resetAngleRotated();

      new Thread( new TelemetryDashboardUpdater() ).start();
      new Thread( new PdpCurrentMonitor() ).start();
      // new Thread( new TalonEncoderTester() ).start();

      SmartDashboard.putBoolean( "usingVision", false );

      logger.info( "leaving robotInit()" );
   }


   /**
    *
    * @param left - talon who leads
    * @param right - talon who follows
    */
   private void configureHangerMotors( CANTalon left, CANTalon right )
   {
      /* set the peak and nominal outputs, 12V means full */
      left.configNominalOutputVoltage( +0.0f, -0.0f );
      left.configPeakOutputVoltage( +12.0f, -12.0f );

      // set up master-slave relationship
      right.changeControlMode( CANTalon.TalonControlMode.Follower );
      right.set( left.getDeviceID() );
   }


   /**
    *
    * @param left - talon who follows
    * @param right - talon w/ encoder who leads
    */
   @SuppressWarnings( "unused" )
   private void configureShooterMotors( CANTalon left, CANTalon right )
   {
      /* first choose the sensor */
      right.setFeedbackDevice( FeedbackDevice.QuadEncoder );
      right.reverseSensor( false );
      right.configEncoderCodesPerRev( 1024 ); // FeedbackDevice.QuadEncoder
      right.setEncPosition( 0 );

      /* set the peak and nominal outputs, 12V means full */
      right.configNominalOutputVoltage( +0.0f, -0.0f );
      right.configPeakOutputVoltage( +12.0f, -12.0f );

      // set up master-slave relationship
      left.changeControlMode( CANTalon.TalonControlMode.Follower );
      left.set( right.getDeviceID() );
   }


   private void configureChassisEncoders( CANTalon left, CANTalon right )
   {
      left.setFeedbackDevice( FeedbackDevice.CtreMagEncoder_Relative );
      left.reverseSensor( false );
      left.configEncoderCodesPerRev( 1024 ); // if using
      // FeedbackDevice.QuadEncoder
      left.setEncPosition( 0 );
      // _talon.configPotentiometerTurns(XXX), // if using
      // FeedbackDevice.AnalogEncoder or AnalogPot

      right.setFeedbackDevice( FeedbackDevice.CtreMagEncoder_Relative );
      right.reverseSensor( true );
      right.configEncoderCodesPerRev( 1024 ); // if using
      // FeedbackDevice.QuadEncoder
      right.setEncPosition( 0 );
      // _talon.configPotentiometerTurns(XXX), // if using
      // FeedbackDevice.AnalogEncoder or AnalogPot
   }

   private static final String autoDoNothing = "Do Nothing";
   private static final String autoSimpleDriveDist = "Simple Drive (Dist)";
   private static final String autoSimpleDriveTime = "Simple Drive (Time)";
   private static final String autoCenter = "Center";
   private static final String autoRight = "Right";
   private static final String autoTuning = "PID Tuning";
   private static final Boolean autoGearFalse = false;
   private static final Boolean autoGearTrue = true;

   private static final Boolean autoShootFalse = false;
   private static final Boolean autoShootTrue = true;

   private static final Boolean visionSaveChooserFalse = false;
   private static final Boolean visionSaveChooserTrue = true;

   private static final Boolean useVisionChooserFalse = false;
   private static final Boolean useVisionChooserTrue = true;

   private static final Boolean useGearSwitchChooserFalse = false;
   private static final Boolean useGearSwitchChooserTrue = true;


   private void initChoosers()
   {
      // Default is to do nothing in autonomous
      autoChooser.addDefault( "Do Nothing", autoDoNothing );
      autoChooser.addObject( "Simple Drive (Dist)", autoSimpleDriveDist );
      autoChooser.addObject( "Simple Drive (Time)", autoSimpleDriveTime );
      autoChooser.addObject( "Center", autoCenter );
      autoChooser.addObject( "Right Shoot", autoRight );
      autoChooser.addObject( "PID Tuning", autoTuning );
      SmartDashboard.putData( "Auto Select", autoChooser );

      // Default is to place gear at the end
      autoGearPlaceChooser.addDefault( "Yes", autoGearTrue );
      autoGearPlaceChooser.addObject( "No", autoGearFalse );
      SmartDashboard.putData( "Auto Gear", autoGearPlaceChooser );

      // Default is not to shoot at the end
      autoShootChooser.addDefault( "No", autoShootFalse );
      autoShootChooser.addObject( "Yes", autoShootTrue );
      SmartDashboard.putData( "Auto Shooter", autoShootChooser );

      // Default is not to save vision files
      visionSaveChooser.addDefault( "No", visionSaveChooserFalse );
      visionSaveChooser.addObject( "Yes", visionSaveChooserTrue );
      SmartDashboard.putData( "Save Images", visionSaveChooser );

      // Default is not to use vision in running autonomous
      useVisionChooser.addDefault( "No", useVisionChooserFalse );
      useVisionChooser.addObject( "Yes", useVisionChooserTrue );
      SmartDashboard.putData( "Use Vision", useVisionChooser );

      // Default is not to use gear Knock
      useGearSwitchChooser.addDefault( "No", useGearSwitchChooserFalse );
      useGearSwitchChooser.addObject( "Yes", useGearSwitchChooserTrue );
      SmartDashboard.putData( "Use Gear Switch", useGearSwitchChooser );
   }


   private void initCameraNetworkTables()
   {
      if ( !cameraTable.isConnected() )
      {
         logger.error( "unable to connect to camera network table" );
         SmartDashboard.putBoolean( "cameraTable", false );
      }
      else
      {
         SmartDashboard.putBoolean( "cameraTable", true );
      }

      vision = new CameraUpdateListener();
      cameraTable.addTableListener( vision );

      setSaveImage( false );
   }


   private void setSaveImage( boolean save )
   {
      cameraTable.putBoolean( "save", save );
      SmartDashboard.putBoolean( "saveImage", save );
   }


   /*
    * (non-Javadoc)
    *
    * @see edu.wpi.first.wpilibj.SampleRobot#disabled()
    */
   @Override
   protected void disabled()
   {
      logger.info( "entering disabled()" );

      drive.arcadeDrive( 0.0, 0.0 );

      rightShooterMotor.changeControlMode( TalonControlMode.PercentVbus );
      rightShooterMotor.set( 0 );

      leftHangerMotor.set( 0.0 );

      setSaveImage( false );

      logger.info( "leaving disabled()" );
   }


   @SuppressWarnings( "unused" )
   private void updateCANPIDValues( CANTalon talon, int profile, double f,
      double p, double i, double d )
   {
      talon.setProfile( profile );
      talon.setF( f ); // 0.05
      talon.setP( p ); // 0.048
      talon.setI( i ); // 0.0004
      talon.setD( d ); // 0.1
   }

   private int shooterPIDCount = 0;
   private final StringBuilder pidBuf = new StringBuilder();


   @SuppressWarnings( "unused" )
   private void logCANShooterPID( CANTalon master, CANTalon slave,
      double targetSpeed )
   {
      if ( !logger.isTraceEnabled() )
      {
         return;
      }

      if ( ++shooterPIDCount < 10 )
      {
         return;
      }
      shooterPIDCount = 0;

      final double masterMotorOutput =
         master.getOutputVoltage() / master.getBusVoltage();
      /* prepare line to print */
      pidBuf.append( "\tout:" );
      pidBuf.append( format( masterMotorOutput, 5 ) );
      pidBuf.append( "\tspd:" );
      pidBuf.append( format( master.getSpeed(), 5 ) );
      final double slaveMotorOutput =
         slave.getOutputVoltage() / slave.getBusVoltage();
      pidBuf.append( "\tslaveOut:" );
      pidBuf.append( format( slaveMotorOutput, 5 ) );
      pidBuf.append( "\terr:" );
      pidBuf.append( master.getClosedLoopError() );
      pidBuf.append( "\ttrg:" );
      pidBuf.append( targetSpeed );
      logger.trace( "shooter PID: {}", pidBuf.toString() );
      pidBuf.setLength( 0 );
   }


   private void openGearFlaps()
   {
      leftGearFlapServo.setAngle( 120 ); // 110
      rightGearFlapServo.setAngle( 50 ); // 60
   }


   private void closeGearFlaps()
   {
      leftGearFlapServo.setAngle( 170 );
      rightGearFlapServo.setAngle( 0 );
   }


   private boolean areGearFlapsClosed()
   {
      final boolean isClosed = ( leftGearFlapServo.getAngle() > 160 );
      SmartDashboard.putBoolean( "gearFlapsClosed", isClosed );
      return isClosed;
   }


   private double getDistanceToGear()
   {
      final double voltage = gearDistanceSensor.getVoltage();
      // if ( logger.isTraceEnabled() )
      // {
      // final double averageVoltage = distanceSensor.getAverageVoltage();
      // logger.trace( "voltage = {}, averageVoltage = {}",
      // format( voltage, 4 ), format( averageVoltage, 4 ) );
      // }

      // from the spec sheet (0.00488 V per 0.19685 in = 40.33811)
      final double distance = ( voltage / 0.00488 ) * 0.19685;
      return distance;
   }


   @SuppressWarnings( "unused" )
   private double getDistanceToBoiler()
   {
      final double voltage = boilerDistanceSensor.getVoltage();
      // if ( logger.isTraceEnabled() )
      // {
      // final double averageVoltage = distanceSensor.getAverageVoltage();
      // logger.trace( "voltage = {}, averageVoltage = {}",
      // format( voltage, 4 ), format( averageVoltage, 4 ) );
      // }

      // from the spec sheet (0.00488 V per 0.19685 in = 40.33811)
      final double distance = ( voltage / 0.00488 ) * 0.19685;
      return distance;
   }

   // private double distanceDriven;


   private int getLeftDriveCount()
   {
      // Needs to be positive forward (shooter/hanger end)
      //
      // CANTalon handles this (but not yet)
      final int count = -leftCANEncoder.getEncPosition();
      SmartDashboard.putNumber( "leftDriveCount:", count );
      return count;
   }


   private int getRightDriveCount()
   {
      // Needs to be positive forward (shooter/hanger end)
      //
      // CANTalon handles this
      final int count = rightCANEncoder.getEncPosition();
      SmartDashboard.putNumber( "rightDriveCount:", count );
      return count;
   }


   private int getAverageDriveCount( int leftCount, int rightCount )
   {
      // Spray & Pray has both encoders now
      return ( leftCount + rightCount ) / 2;
   }


   private void resetDistanceDriven()
   {
      leftCANEncoder.setEncPosition( 0 );
      rightCANEncoder.setEncPosition( 0 );

      getLeftDriveCount();
      getRightDriveCount();

      SmartDashboard.putNumber( "distanceDriven:", getDistanceDriven() );
   }


   private double getDistanceDriven()
   {
      final int leftDriveCount = getLeftDriveCount();
      final int rightDriveCount = getRightDriveCount();

      // Spray & Pray (wheel encoders)
      // final double clicksPer10Feet = 1562;
      // Spray & Pray (CANTalon encoders)
      final double clicksPer10Feet = 219289;
      final double distanceDriven =
         getAverageDriveCount( leftDriveCount, rightDriveCount )
            * ( ( 10.0 / clicksPer10Feet ) * 12.0 );
      SmartDashboard.putNumber( "distanceDriven:", distanceDriven );
      return Math.abs( distanceDriven );
   }

   private double angleRotated;


   private void resetAngleRotated()
   {
      ahrs.zeroYaw();

      angleRotated = 0.0;
      SmartDashboard.putNumber( "angleRotated", angleRotated );
   }


   private double getAngleRotated()
   {
      final double rotation = ahrs.getAngle();
      angleRotated = rotation;
      SmartDashboard.putNumber( "angleRotated", rotation );
      return angleRotated;
   }


   private boolean isGearOnPeg()
   {
      final boolean gearOnPeg = gearPegSensor.get();
      SmartDashboard.putBoolean( "gearOnPeg", gearOnPeg );

      return gearOnPeg;
   }


   /*
    * (non-Javadoc)
    *
    * @see edu.wpi.first.wpilibj.SampleRobot#autonomous()
    */
   @Override
   public void autonomous()
   {
      logger.info( "starting autonomous program" );

      setSaveImage( visionSaveChooser.getSelected() );

      final Alliance alliance = DriverStation.getInstance().getAlliance();
      logger.info( "alliance = {}", alliance );

      resetDashboard( 0.0, 0.0 );

      final String autoSelected = autoChooser.getSelected();
      SmartDashboard.putString( "autoSelected", autoSelected );
      logger.info( "autoSelected = {}", autoSelected );
      final Boolean autoGearPlace = autoGearPlaceChooser.getSelected();
      SmartDashboard.putBoolean( "autoGearPlace", autoGearPlace );
      logger.info( "autoGearPlace = {}", autoGearPlace );
      final Boolean autoShoot = autoShootChooser.getSelected();
      SmartDashboard.putBoolean( "autoShoot", autoShoot );
      logger.info( "autoShoot = {}", autoShoot );

      final Boolean useVision = useVisionChooser.getSelected();
      logger.info( "useVision = {}", useVision );
      if ( useVision )
      {
         new Thread( new AutonomousVisionLogger() ).start();
      }

      switch ( autoSelected )
      {
      case autoDoNothing:
         autonomousDoNothing();
         break;

      case autoSimpleDriveDist:
         autonomousSimpleDriveDist();
         break;

      case autoSimpleDriveTime:
         autonomousSimpleDriveTimed();
         break;

      case autoCenter:
         autonomousCenter();
         break;

      case autoRight:
         switch ( alliance )
         {
         case Red:
            // autonomousGearAndShooter( autoPlace, -1 );
            if ( useVision )
            {
               autonomousGearWithVision( autoGearPlace, -1 );
            }
            else
            {
               autonomousGear( autoGearPlace, -1 );
            }
            break;
         case Blue:
            if ( useVision )
            {
               autonomousGearWithVision( autoGearPlace, -1 );
            }
            else
            {
               autonomousGear( autoGearPlace, -1 );
            }
            break;
         case Invalid:
         default:
            logger.error( "Uknown value of enumeration: {}", alliance );
            break;
         }
         break;

      case autoTuning:
         autonomousTune();
         break;

      default:
         logger.error( "Unknown auto mode: {}", autoSelected );
         break;
      }

      // Make sure we are stopped when we leave to return
      drive.arcadeDrive( 0.0, 0.0 );

      // Hang out until done ...
      while ( isEnabled() && isAutonomous() )
      {
         sleep( 5 );
      }

      logger.info( "leaving autonomous program" );
   }

   @SuppressWarnings( "unused" )
   private class SpinHanger
      implements Runnable
   {

      // delay time (msec)
      private final long delayTime;
      // spin time (msec)
      private final long spinTime;
      // speed to turn motor (must be negative!)
      private final double speed;


      public SpinHanger( long delayTime, long spinTime, double speed )
      {
         this.delayTime = delayTime;
         this.spinTime = spinTime;
         // Need to set to negative value
         this.speed = ( speed > 0 ) ? -speed : speed;
      }


      /*
       * (non-Javadoc)
       *
       * @see java.lang.Runnable#run()
       */
      @Override
      public void run()
      {
         logger.info( "starting SpinHanger.run" );
         sleep( delayTime );

         leftHangerMotor.set( speed );
         sleep( spinTime );
         leftHangerMotor.set( 0.0 );

         logger.info( "exiting SpinHanger.run" );
      }
   }


   private void autonomousDoNothing()
   {
      logger.info( "entering autonomousDoNothing" );
      SmartDashboard.putBoolean( "usingVision", false );

      resetDashboard( 0.0, 0.0 );

      drive.arcadeDrive( 0.00, 0.00 );

      while ( isEnabled() && isAutonomous() )
      {
         sleep( 5 );
      }

      drive.arcadeDrive( 0.00, 0.00 );

      logger.info( "leaving autonomousDoNothing" );
   }


   private void autonomousSimpleDriveDist()
   {
      logger.info( "entering autonomousSimpleDriveDist" );
      SmartDashboard.putBoolean( "usingVision", false );

      final double distanceToDrive = 5 * 12; // hard coded maybe change
      resetDashboard( distanceToDrive, 0.0 );

      drive.arcadeDrive( -0.50, 0.00 );

      while ( isEnabled() && isAutonomous() )
      {
         final double distanceDriven = getDistanceDriven();
         if ( distanceDriven >= distanceToDrive )
         {
            break;
         }

         final double angle = ahrs.getAngle();
         final double turn = angle * 0.15;
         drive.arcadeDrive( -0.50, turn );
      }

      // Backdrive to stop
      drive.arcadeDrive( 0.20, 0.00 );
      //
      sleep( 200 );

      drive.arcadeDrive( 0.00, 0.00 );

      logger.info( "leaving autonomousSimpleDriveDist" );
   }


   private void autonomousSimpleDriveTimed()
   {
      logger.info( "entering autonomousSimpleDriveTimed" );
      SmartDashboard.putBoolean( "usingVision", false );

      final double distanceToDrive = 5 * 12; // hard coded maybe change
      resetDashboard( distanceToDrive, 0.0 );

      drive.arcadeDrive( -0.50, 0.00 );

      sleep( 3000 ); // 5 seconds

      // Let it coast stop

      drive.arcadeDrive( 0.00, 0.00 );

      // final long startTime = System.currentTimeMillis();
      // while ( isEnabled() && isAutonomous() )
      // {
      // final long currentTime = System.currentTimeMillis();
      // if ( Math.abs( startTime - currentTime ) > 3000 )
      // {
      // break;
      // }
      //
      // final double angle = ahrs.getAngle();
      // final double turn = angle * 0.15;
      // drive.arcadeDrive( -0.50, turn );
      // }

      logger.info( "leaving autonomousSimpleDriveTimed" );
   }


   private void autonomousCenter()
   {
      logger.info( "entering autonomousCenter" );
      SmartDashboard.putBoolean( "usingVision", false );

      final double distanceToDrive = getDistanceToGear() - 12;
      resetDashboard( distanceToDrive, 0.0 );
      //
      driveStraightAndHangGear( true, distanceToDrive );

      drive.arcadeDrive( 0.00, 0.00 );

      logger.info( "leaving autonomousCenter" );
   }


   private void autonomousGear( boolean autoPlace, int rotationDirection )
   {
      logger.info( "entering autonomousGear autoPlace={}, rotationDirection={}",
         autoPlace, rotationDirection );
      SmartDashboard.putBoolean( "usingVision", false );

      double distanceToDrive = 3 * 12;
      resetDashboard( distanceToDrive, 0.0 );
      //
      driveToDistance( distanceToDrive );

      final double angleToRotate = 60 * rotationDirection;
      resetDashboard( 0.0, angleToRotate );
      //
      rotateWithPID( angleToRotate );

      distanceToDrive = getDistanceToGear() - 12;
      resetDashboard( distanceToDrive, 0.0 );
      //
      driveStraightAndHangGear( autoPlace, distanceToDrive );

      drive.arcadeDrive( 0.00, 0.00 );

      logger.info( "leaving autonomousGear" );
   }


   private void autonomousGearWithVision( boolean autoPlace,
      int rotationDirection )
   {
      logger.info(
         "entering autonomousGearWithVision autoPlace={}, rotationDirection={}",
         autoPlace, rotationDirection );
      SmartDashboard.putBoolean( "usingVision", false );

      double distanceToDrive = 3 * 12;
      resetDashboard( distanceToDrive, 0.0 );
      //
      driveToDistance( distanceToDrive );

      final double angleToRotate = 40 * rotationDirection;
      resetDashboard( 0.0, angleToRotate );
      //
      rotateWithPID( angleToRotate );

      alignWithVision();

      // vision stops working at 18 inches
      distanceToDrive = getDistanceToGear() - 24;
      resetDashboard( distanceToDrive, 0.0 );
      //
      driveStraightWithVisionAndHangGear( autoPlace, distanceToDrive );

      drive.arcadeDrive( 0.00, 0.00 );

      logger.info( "leaving autonomousGearWithVision" );
   }


   private boolean waitForGearSwitch()
   {
      boolean found = false;
      while ( isEnabled() && isAutonomous() )
      {
         if ( isGearOnPeg() )
         {
            logger.info( "exiting Gear on Peg" );
            found = true;
            break;
         }

         sleep( 50 );
      }

      return found;
   }


   @SuppressWarnings( "unused" )
   private void autonomousGearAndShooter( boolean autoPlace,
      int rotationDirection )
   {
      logger.info(
         "entering autonomousShooter autoPlace={}, rotationDirection={}",
         autoPlace, rotationDirection );
      SmartDashboard.putBoolean( "usingVision", false );

      double distanceToDrive = 3 * 12;
      resetDashboard( distanceToDrive, 0.0 );
      //
      driveToDistance( distanceToDrive );

      final double angleToRotate = 60 * rotationDirection;
      resetDashboard( 0.0, angleToRotate );
      //
      rotate( angleToRotate );

      distanceToDrive = getDistanceToGear() - 12;
      resetDashboard( distanceToDrive, 0.0 );
      //
      driveStraightAndHangGear( autoPlace, distanceToDrive );

      distanceToDrive = 36;// getDistanceToBoiler() - 12;
      resetDashboard( distanceToDrive, 0.0 );
      //
      driveStraightAndShoot( distanceToDrive );

      drive.arcadeDrive( 0.00, 0.00 );

      logger.info( "leaving autonomousShooter" );
   }


   private void autonomousTune()
   {
      logger.info( "entering autonomousTune" );

      resetDistanceDriven();
      resetAngleRotated();

      // PID Tuning
      final double fValue = prefs.getDouble( "tuneFValue", 0.00 );
      final double pValue = prefs.getDouble( "tunePValue", 0.00 );
      final double iValue = prefs.getDouble( "tuneIValue", 0.0000 );
      final double dValue = prefs.getDouble( "tuneDValue", 0.0 );
      logger.debug( "fValue = {}, pValue = {}, iValue = {}, dValue = {}",
         fValue, pValue, iValue, dValue );
      final double targetValue = prefs.getDouble( "tuneTarget", 0.0 );
      logger.debug( "targetValue = {}", targetValue );

      SmartDashboard.putNumber( "tuneTarget", targetValue );

      final GyroRotatePID pid =
         new GyroRotatePID( fValue, pValue, iValue, dValue );
      pid.setSetpoint( targetValue );
      pid.enable();
      while ( /* !pid.onTarget() && */ isEnabled() && isAutonomous() )
      {
         SmartDashboard.putNumber( "pidError", pid.getError() );
      }
      pid.disable();

      drive.arcadeDrive( 0.00, 0.00 );

      logger.info( "leaving autonomousTune" );
   }


   private void resetDashboard( double targetDistance, double targetAngle )
   {
      resetDistanceDriven();
      SmartDashboard.putNumber( "targetDistance", targetDistance );
      resetAngleRotated();
      SmartDashboard.putNumber( "targetAngle", targetAngle );
   }


   private void driveToDistance( double distance )
   {
      logger.info( "entering driveToDistance distance={}", distance );
      SmartDashboard.putNumber( "targetDistance", distance );
      SmartDashboard.putBoolean( "usingVision", false );

      resetDistanceDriven();
      resetAngleRotated();

      final double angle = ahrs.getAngle();
      final double turn = angle * 0.15;
      drive.arcadeDrive( -0.50, turn );

      while ( isEnabled() && isAutonomous() )
      {
         final double distanceDriven = getDistanceDriven();

         if ( distanceDriven >= distance )
         {
            break;
         }
      }

      drive.arcadeDrive( 0.00, 0.00 );

      logger.info( "leaving driveToDistance" );
   }


   private void driveStraightAndHangGear( boolean autoPlace, double distance )
   {
      logger.info(
         "entering driveStraightAndHangGear autoPlace={}, distance={}",
         autoPlace, distance );
      SmartDashboard.putNumber( "targetDistance", distance );
      SmartDashboard.putBoolean( "usingVision", false );

      resetDistanceDriven();
      resetAngleRotated();

      // Sleep for Auto Drive Distance
      sleep( 500 );

      // Start slow so robot does jump
      drive.arcadeDrive( -0.30, 0.00 );
      sleep( 500 );

      double speedToDrive = 0.60; // 0.85
      drive.arcadeDrive( -speedToDrive, 0.00 );
      while ( isAutonomous() && isEnabled() )
      {
         final double distanceDriven = getDistanceDriven();
         if ( distanceDriven >= ( distance - 12 ) ) // 15
         {
            logger.info( "exiting distanceDriven={}", distanceDriven );
            break;
         }

         final double angle = ahrs.getAngle();
         final double turn = angle * 0.25; // 0.15
         drive.arcadeDrive( -speedToDrive, turn );
         // drive.arcadeDrive( -speedToDrive, 0.0 );
      }

      drive.arcadeDrive( -speedToDrive, 0.00 ); // undo any turn correction

      resetDistanceDriven();

      speedToDrive = 0.40;
      drive.arcadeDrive( -speedToDrive, 0.0 );
      int lastCount = getRightDriveCount();
      while ( isAutonomous() && isEnabled() )
      {
         sleep( 500 );

         final int currentCount = getRightDriveCount();
         if ( lastCount == currentCount )
         {
            logger.info( "exiting countsEqual={}", currentCount );
            final double distanceDriven = getDistanceDriven();
            logger.info( "distanceDriven={}", distanceDriven );
            break;
         }
         else
         {
            lastCount = currentCount;
         }
      }

      drive.arcadeDrive( 0.00, 0.00 );

      final boolean gearSwitch = useGearSwitchChooser.getSelected();
      logger.info( "gearSwitch={}", gearSwitch );
      if ( gearSwitch )
      {
         sleep( 500 );

         final boolean hangGear = waitForGearSwitch();
         autoPlace = ( autoPlace && hangGear );
         logger.info( "corrected autoPlace={}", autoPlace );
      }

      if ( autoPlace && isAutonomous() && isEnabled() )
      {
         resetDistanceDriven();
         logger.info( "reset of distanceDriven={}", getDistanceDriven() );

         openGearFlaps();

         sleep( 1000 );

         logger.info( "reset of distanceDriven={}", getDistanceDriven() );

         drive.arcadeDrive( 0.50, 0.00 ); // 60
         while ( isEnabled() && isAutonomous() )
         {
            final double distanceDriven = getDistanceDriven();
            logger.info( "distanceDriven={}", distanceDriven );
            if ( distanceDriven > 12 ) // 24
            {
               logger.info( "exiting distanceDriven={}", distanceDriven );
               break;
            }
         }

         drive.arcadeDrive( 0.00, 0.00 );

         // closeGearFlaps();
      }

      drive.arcadeDrive( 0.00, 0.00 );

      logger.info( "leaving driveStraightAndHangGear" );
   }


   @SuppressWarnings( "unused" )
   private void drivePostGearHang()
   {
      // // Move After
      // drive.arcadeDrive( 0.0, 0.0 );
      // Sleeping( 100 );
      // drive.arcadeDrive( 0.0, -0.75 ); // Turn Left
      // Sleeping( 600 );
      // drive.arcadeDrive( 0.00, 0.00 );
      // Sleeping( 100 );
      // drive.arcadeDrive( -1.00, 0.0 ); // drive out
      // Sleeping( 500 );
      // drive.arcadeDrive( 0.0, 0.0 );
      // Sleeping( 100 );
      // drive.arcadeDrive( 0.0, 0.0 );
   }


   private void driveStraightWithVisionAndHangGear( boolean autoPlace,
      double distance )
   {
      logger.info(
         "entering driveStraightWithVisionAndHangGear autoPlace={}, distance={}",
         autoPlace, distance );

      resetDistanceDriven();
      resetAngleRotated();
      SmartDashboard.putNumber( "targetDistance", distance );

      drive.arcadeDrive( -0.60, 0.00 );
      while ( isAutonomous() && isEnabled() )
      {
         final double distanceDriven = getDistanceDriven();
         if ( distanceDriven >= ( distance - 12 ) )
         {
            break;
         }

         double angle;
         final boolean visionHasUpdated = vision.hasUpdate();
         SmartDashboard.putBoolean( "usingVision", visionHasUpdated );
         if ( visionHasUpdated )
         {
            angle = vision.getAngle();
         }
         else
         {
            angle = ahrs.getAngle();
         }
         final double turn = angle * 0.15;
         drive.arcadeDrive( -0.60, turn );
      }
      drive.arcadeDrive( -0.60, 0.00 ); // undo any turn correction
      logger.debug( "reached Target Distance" );

      drive.arcadeDrive( -0.30, 0.0 );
      int lastCount = getRightDriveCount();
      while ( isAutonomous() && isEnabled() )
      {
         sleep( 100 );

         final int currentCount = getRightDriveCount();
         if ( lastCount == currentCount )
         {
            logger.debug( "counts equal, so stopped" );
            break;
         }
         else
         {
            lastCount = currentCount;
         }
      }

      drive.arcadeDrive( 0.00, 0.00 );

      if ( autoPlace && isAutonomous() && isEnabled() )
      {
         openGearFlaps();
         sleep( 1000 );

         resetDistanceDriven();

         drive.arcadeDrive( 0.35, 0.00 );
         while ( isEnabled() && isAutonomous() )
         {
            final double distanceDriven = getDistanceDriven();
            if ( distanceDriven > 12 )
            {
               break;
            }
         }
         drive.arcadeDrive( 0.00, 0.00 );

         closeGearFlaps();
      }

      drive.arcadeDrive( 0.00, 0.00 );

      logger.info( "leaving driveStraightWithVisionAndHangGear" );
   }


   private void driveStraightAndShoot( double distance )
   {
      logger.info( "entering driveStraightAndShoot distance={}", distance );
      SmartDashboard.putNumber( "targetDistance", distance );
      SmartDashboard.putBoolean( "usingVision", false );

      leftHangerMotor.set( -0.75 );
      int x = 1;
      while ( isEnabled() && isAutonomous() )
      {
         sleep( 50 );

         x++;
         if ( x >= 20 )
         {
            break;
         }
      }
      leftHangerMotor.set( 0.00 );

      // PID Tuning
      final double fValue = prefs.getDouble( "shooterFValue", 0.05 );
      final double pValue = prefs.getDouble( "shooterPValue", 0.048 );
      final double iValue = prefs.getDouble( "shooterIValue", 0.0004 );
      final double dValue = prefs.getDouble( "shooterDValue", 0.1 );
      logger.info( "fValue = {}, pValue = {}, iValue = {}, dValue = {}", fValue,
         pValue, iValue, dValue );
      /* set closed loop gains in slot0 */
      // updatePIDValues( rightShooterMotor, 0, fValue, pValue, iValue, dValue
      // );

      final double rpmValue = prefs.getDouble( "shooterRpmValue", -4425.0 );
      logger.info( "rpmValue = {}", rpmValue );

      resetDistanceDriven();
      resetAngleRotated();

      // drive.arcadeDrive( 0.60, 0.00 );
      // while ( isAutonomous() && isEnabled() )
      // {
      // final double distanceDriven = getDistanceDriven();
      // SmartDashboard.putNumber( "distanceDriven:", distanceDriven );
      // if ( distanceDriven >= ( distance - 12 ) )
      // {
      // break;
      // }
      //
      // final double angle = ahrs.getAngle();
      // final double turn = angle * 0.15;
      // drive.arcadeDrive( 0.60, turn );
      // }
      // logger.debug( "reached Target Distance" );
      //
      // drive.arcadeDrive( 0.30, 0.0 );
      // int lastCount = getRightDriveCount();
      // while ( isAutonomous() && isEnabled() )
      // {
      // try
      // {
      // Thread.sleep( 1000 );
      // }
      // catch ( final InterruptedException ex )
      // {
      // //
      // }
      //
      // final int currentCount = getRightDriveCount();
      // if ( lastCount == currentCount )
      // {
      // logger.debug( "counts equal" );
      // break;
      // }
      // else
      // {
      // lastCount = currentCount;
      // }
      //
      // }

      drive.arcadeDrive( 0.00, 0.00 );

      // rightShooterMotor.changeControlMode( TalonControlMode.Speed );
      // rightShooterMotor.set( rpmValue ); // RPM
      // leftShooterMotor slaved to right
      // logShooterPID( rightShooterMotor, leftShooterMotor, rpmValue );
      sleep( 2000 );

      resetDistanceDriven();
      // drive.arcadeDrive( -0.35, 0.00 );
      //
      // while ( isEnabled() && isAutonomous() )
      // {
      // final double distanceDriven = getDistanceDriven();
      // SmartDashboard.putNumber( "distanceDriven:", distanceDriven );
      // if ( distanceDriven < 18 )
      // {
      // break;
      // }
      // }
      // drive.arcadeDrive( 0.00, 0.00 );

      // shooterFeederMotor.set( Relay.Value.kOn );

      x = 1;
      while ( isEnabled() && isAutonomous() )
      {
         sleep( 50 );

         x++;
         if ( x >= 100 )
         {
            break;
         }
      }

      // shooterFeederMotor.set( Relay.Value.kOff );

      // rightShooterMotor.changeControlMode( TalonControlMode.PercentVbus );
      // rightShooterMotor.set( 0 );

      drive.arcadeDrive( 0.00, 0.00 );

      logger.info( "leaving driveStraightAndShoot" );
   }


   private void rotate( double angle )
   {
      logger.info( "entering rotate angle={}", angle );
      SmartDashboard.putNumber( "targetAngle", angle );
      SmartDashboard.putBoolean( "usingVision", false );

      resetDistanceDriven();
      resetAngleRotated();

      while ( isEnabled() && isAutonomous() )
      {
         final double rotation = getAngleRotated();

         if ( Math.abs( rotation - angle ) < 2 )
         {
            break;
         }
         double turn = ( rotation - angle ) * 0.15;
         if ( turn > 0.80 )
         {
            turn = 0.80;
         }
         else if ( turn < -0.80 )
         {
            turn = -0.80;
         }
         drive.arcadeDrive( 0.0, turn );
      }

      drive.arcadeDrive( 0.00, 0.00 );

      logger.info( "leaving rotate" );
   }

   private class YawPIDSource
      implements PIDSource
   {

      /*
       * (non-Javadoc)
       *
       * @see edu.wpi.first.wpilibj.PIDSource#setPIDSourceType(edu.wpi.first.
       * wpilibj. PIDSourceType)
       */
      @Override
      public void setPIDSourceType( PIDSourceType pidSource )
      {
         // Why would anyone call this?
      }


      /*
       * (non-Javadoc)
       *
       * @see edu.wpi.first.wpilibj.PIDSource#getPIDSourceType()
       */
      @Override
      public PIDSourceType getPIDSourceType()
      {
         return PIDSourceType.kDisplacement;
      }


      /*
       * (non-Javadoc)
       *
       * @see edu.wpi.first.wpilibj.PIDSource#pidGet()
       */
      @Override
      public double pidGet()
      {
         return ahrs.getAngle();
      }

   }

   private class DriveTurnPIDOutput
      implements PIDOutput
   {

      /*
       * (non-Javadoc)
       *
       * @see edu.wpi.first.wpilibj.PIDOutput#pidWrite(double)
       */
      @Override
      public void pidWrite( double output )
      {
         // Change sign to make robot go the way we want
         drive.arcadeDrive( 0.0, -output );
      }

   }

   private class GyroRotatePID
      extends PIDController
   {

      private static final double Kf = 0.0;
      private static final double Kp = 0.0;
      private static final double Ki = 0.0;
      private static final double Kd = 0.0;


      public GyroRotatePID( double angle )
      {
         super( Kp, Ki, Kd, Kf, new YawPIDSource(), new DriveTurnPIDOutput(),
            0.050 );
         intialize();

         setSetpoint( angle );
      }


      public GyroRotatePID( double fValue, double pValue, double iValue,
         double dValue )
      {
         super( pValue, iValue, dValue, fValue, new YawPIDSource(),
            new DriveTurnPIDOutput(), 0.050 );
         intialize();

      }


      private void intialize()
      {
         setAbsoluteTolerance( 1.0 );
         setOutputRange( -0.5, 0.5 );
         setInputRange( -90, 90 );
      }

   }

   private class VisionPIDSource
      implements PIDSource
   {

      /*
       * (non-Javadoc)
       *
       * @see edu.wpi.first.wpilibj.PIDSource#setPIDSourceType(edu.wpi.first.
       * wpilibj. PIDSourceType)
       */
      @Override
      public void setPIDSourceType( PIDSourceType pidSource )
      {
         // Why would anyone call this?
      }


      /*
       * (non-Javadoc)
       *
       * @see edu.wpi.first.wpilibj.PIDSource#getPIDSourceType()
       */
      @Override
      public PIDSourceType getPIDSourceType()
      {
         return PIDSourceType.kDisplacement;
      }

      private double visionAngle = 0;


      /*
       * (non-Javadoc)
       *
       * @see edu.wpi.first.wpilibj.PIDSource#pidGet()
       */
      @Override
      public double pidGet()
      {
         final boolean update = vision.hasUpdate();
         if ( update )
         {
            visionAngle = vision.getAngle();
         }
         logger.trace( "return update={} and angle={}", update,
            format( visionAngle, 1 ) );
         return visionAngle;
      }

   }


   private void rotateWithPID( double angle )
   {
      logger.info( "entering rotateWithPID angle={}", angle );
      SmartDashboard.putNumber( "targetAngle", angle );

      resetDistanceDriven();
      resetAngleRotated();

      final double fValue = prefs.getDouble( "tuneFValue", 0.00 );
      final double pValue = prefs.getDouble( "tunePValue", 0.00 );
      final double iValue = prefs.getDouble( "tuneIValue", 0.0000 );
      final double dValue = prefs.getDouble( "tuneDValue", 0.0 );
      logger.info( "fValue = {}, pValue = {}, iValue = {}, dValue = {}", fValue,
         pValue, iValue, dValue );

      final GyroRotatePID pid =
         new GyroRotatePID( fValue, pValue, iValue, dValue );
      pid.setSetpoint( angle );

      pid.enable();

      while ( !pid.onTarget() && isEnabled() && isAutonomous() )
      {
         SmartDashboard.putNumber( "angleRotated", angle - pid.getError() );
      }

      pid.disable();

      logger.info( "leaving rotateWithPID angle={}", angle );
   }

   private class VisionAlignPID
      extends PIDController
   {

      public VisionAlignPID( double fValue, double pValue, double iValue,
         double dValue )
      {
         super( pValue, iValue, dValue, fValue, new VisionPIDSource(),
            new DriveTurnPIDOutput(), 0.050 );
         intialize();
      }


      private void intialize()
      {
         setAbsoluteTolerance( 1.0 );
         setOutputRange( -0.5, 0.5 );
         setInputRange( -45, 45 );
      }

   }


   private void alignWithVision()
   {
      logger.info( "entering alignWithVision" );

      resetDistanceDriven();
      resetAngleRotated();

      final double fValue = prefs.getDouble( "tuneFValue", 0.00 );
      final double pValue = prefs.getDouble( "tunePValue", 0.00 );
      final double iValue = prefs.getDouble( "tuneIValue", 0.0000 );
      final double dValue = prefs.getDouble( "tuneDValue", 0.0 );
      logger.info( "fValue = {}, pValue = {}, iValue = {}, dValue = {}", fValue,
         pValue, iValue, dValue );

      final VisionAlignPID pid =
         new VisionAlignPID( fValue, pValue, iValue, dValue );

      pid.enable();

      while ( !pid.onTarget() && isEnabled() && isAutonomous() )
      {
         SmartDashboard.putNumber( "angleRotated", pid.getError() );
      }

      pid.disable();

      logger.info( "leaving alignWithVision" );
   }


   /*
    * (non-Javadoc)
    *
    * @see edu.wpi.first.wpilibj.SampleRobot#operatorControl()
    */
   @Override
   public void operatorControl()
   {
      logger.info( "starting operatorControl program" );

      resetDashboard( 0.0, 0.0 );

      setSaveImage( visionSaveChooser.getSelected() );

      // PID Tuning
      final double fValue = prefs.getDouble( "shooterFValue", 0.05 );
      final double pValue = prefs.getDouble( "shooterPValue", 0.048 );
      final double iValue = prefs.getDouble( "shooterIValue", 0.0004 );
      final double dValue = prefs.getDouble( "shooterDValue", 0.1 );
      logger.info( "fValue = {}, pValue = {}, iValue = {}, dValue = {}", fValue,
         pValue, iValue, dValue );
      /* set closed loop gains in slot0 */
      // updatePIDValues( rightShooterMotor, 0, fValue, pValue, iValue, dValue
      // );

      final double rpmValue = prefs.getDouble( "shooterRpmValue", -4425.0 );
      logger.info( "rpmValue = {}", rpmValue );

      boolean lastSwapButton = false;
      boolean gearDirection = true;
      boolean shooterDirection = false;
      final String gearText = "Gear Direction";
      final String shooterText = "Shooter Direction";
      String direction = gearDirection ? gearText : shooterText;
      SmartDashboard.putString( "direction", direction );
      SmartDashboard.putBoolean( "directionFlag", gearDirection );

      boolean gearOpen = false;
      final String gearHangText = "Gear Hang";
      final String gearLoadText = "Gear Load";
      String gear = gearOpen ? gearHangText : gearLoadText;
      SmartDashboard.putString( "gear", gear );
      SmartDashboard.putBoolean( "gearFlag", gearOpen );

      boolean hangerLimited = false;
      double hangerSpeedLimited = resetHangerSpeed();
      SmartDashboard.putBoolean( "hangerLimited", hangerLimited );
      SmartDashboard.putNumber( "hangerSpeed", hangerSpeedLimited );

      while ( isOperatorControl() && isEnabled() )
      {

         /*
          *************************************************
          * OPERATOR
          *************************************************
          */

         /*
          * Turrent Shooter
          */

         // turret.fireShooter( operatorStick.getRawButton( 2 ) );

         /*
          * Gear Load & Unload
          */

         // Left Bumper
         final boolean gearButton = operatorStick.getRawButton( 5 );
         gearOpen = gearButton;
         gear = gearOpen ? gearHangText : gearLoadText;
         SmartDashboard.putString( "gear", gear );
         SmartDashboard.putBoolean( "gearFlag", gearOpen );

         if ( gearButton )
         {
            openGearFlaps();
         }
         else
         {
            closeGearFlaps();
         }

         /*
          * Hanging
          */

         final double voltage = leftHangerMotor.getOutputVoltage();
         final double current = leftHangerMotor.getOutputCurrent();

         // Right Trigger
         final double hangSpeed = operatorStick.getRawAxis( 3 );
         // Right Bumper
         final boolean hangButton = operatorStick.getRawButton( 6 );

         if ( hangButton )
         {
            if ( current > 55.0 )
            {
               hangerLimited = true;
            }
            // if ( !hangerLimited )
            // {
            // System.out.println( "current: " + format( current, 2 )
            // + ", voltage: " + format( voltage, 2 ) + ", hangerLimited="
            // + hangerLimited );
            // }
            // else
            // {
            // System.out.println( "**** current: " + format( current, 2 ) );
            // }

            if ( !hangerLimited )
            {
               hangerSpeedLimited = bumpHangerSpeed( hangerSpeedLimited );
            }
            else
            {
               // hold at current position
               hangerSpeedLimited = resetHangerSpeed();
            }
         }
         else
         {
            if ( !isZero( hangSpeed ) )
            {
               if ( current > 55.0 )
               {
                  hangerLimited = true;
               }
               // if ( !hangerLimited )
               // {
               // System.out.println( "current: " + format( current, 2 )
               // + ", voltage: " + format( voltage, 2 ) + ", hangerLimited="
               // + hangerLimited );
               // }

               if ( !hangerLimited )
               {
                  hangerSpeedLimited = hangSpeed;
               }
               else
               {
                  // hold at current position
                  hangerSpeedLimited = resetHangerSpeed();
               }
            }
            else
            {
               hangerLimited = false;
               hangerSpeedLimited = resetHangerSpeed();
            }
         }
         leftHangerMotor.set( -hangerSpeedLimited ); // right is slaved
         SmartDashboard.putBoolean( "hangerLimited", hangerLimited );
         SmartDashboard.putNumber( "hangerSpeed", hangerSpeedLimited );

         /*
          *************************************************
          * DRIVER
          *************************************************
          */

         /*
          * Driving Direction
          */

         // Blue Button
         final boolean swapButton = driverStick.getRawButton( 3 );
         if ( swapButton != lastSwapButton )
         {
            lastSwapButton = swapButton;
            if ( swapButton )
            {
               gearDirection = !gearDirection;
               shooterDirection = !shooterDirection;

               direction = gearDirection ? gearText : shooterText;
               SmartDashboard.putString( "direction", direction );
               SmartDashboard.putBoolean( "directionFlag", gearDirection );

               // attempt to smooth transition (needs to be own thread)
               drive.arcadeDrive( 0.0, 0.0 );
               sleep( 100 );
            }
         }

         /*
          * Driving
          */

         // Left Stick Y-axis
         double speed = driverStick.getRawAxis( 1 );
         // Right Stick X-axis
         double turn = driverStick.getRawAxis( 4 );
         // Right Bumper
         final boolean turboButtonRight = driverStick.getRawButton( 6 );
         // Left Bumper
         final boolean turboButtonLeft = driverStick.getRawButton( 5 );
         SmartDashboard.putNumber( "hmiSpeed:", speed );
         SmartDashboard.putNumber( "hmiTurn:", turn );
         SmartDashboard.putBoolean( "hmiTurbo", turboButtonRight );
         SmartDashboard.putBoolean( "hmiCrawl", turboButtonLeft );

         if ( isZero( speed ) )
         {
            speed = 0.0;
         }
         if ( isZero( turn ) )
         {
            turn = 0.0;
         }

         if ( !turboButtonRight && !turboButtonLeft )
         {
            speed *= 0.60;
            turn *= 0.70;
         }
         if ( shooterDirection )
         {
            speed = -speed;
            // turn stays the same
         }
         SmartDashboard.putNumber( "speed:", speed );
         SmartDashboard.putNumber( "turn:", turn );

         drive.arcadeDrive( speed, -turn );

         sleep( 20 ); // Slow loop down to 20ms
      }

      drive.arcadeDrive( 0.0, 0.0 );

      logger.info( "exiting operatorControl program" );
   }


   private double resetHangerSpeed()
   {
      return 0.20; // default speed to hold at top
   }


   private double bumpHangerSpeed( double oldSpeed )
   {
      final double newSpeed = oldSpeed * 1.05;
      return ( newSpeed > 1.0 ) ? 1.0 : newSpeed;
   }


   // private void bumberBreak( double acc )
   // {
   // while ( true )
   // {
   // final double currentAccX = ahrs.getWorldLinearAccelX();
   // final double currentJerkX = currentAccX - lastAccX;
   // lastAccX = currentAccX;
   // }
   // }

   /**
    * Runs during test mode
    */
   @Override
   public void test()
   {
      logger.info( "starting test program" );

      logger.info( "exiting test program" );
   }

   @SuppressWarnings( "unused" )
   private class VisionAngleDashboardUpdater
      implements Runnable
   {

      /*
       * (non-Javadoc)
       *
       * @see java.lang.Runnable#run()
       */
      @Override
      public void run()
      {
         long lastValidCount = 0;
         while ( true )
         {
            SmartDashboard.putNumber( "cameraAngle", vision.newAngle );
            SmartDashboard.putNumber( "cameraCount", vision.newValidCount );
            final long validCount = (long) vision.newValidCount;
            final boolean valid = ( ( Math.abs( vision.newAngle ) < 90 )
               && ( validCount != lastValidCount ) );
            SmartDashboard.putBoolean( "cameraValid", valid );
            lastValidCount = validCount;

            // Pi updates every 50 msec
            sleep( 200 );
         }
      }

   }

   private class TelemetryDashboardUpdater
      implements Runnable
   {

      /*
       * (non-Javadoc)
       *
       * @see java.lang.Runnable#run()
       */
      @Override
      public void run()
      {
         // Most get methods set the dashboard; so just call them
         while ( true )
         {
            getLeftDriveCount();
            getRightDriveCount();
            getDistanceDriven();

            final double distance = getDistanceToGear();
            SmartDashboard.putNumber( "distanceToWall:", distance );

            SmartDashboard.putNumber( "IMU_Yaw", ahrs.getYaw() );
            SmartDashboard.putNumber( "IMU_Pitch", ahrs.getPitch() );
            SmartDashboard.putNumber( "IMU_Roll", ahrs.getRoll() );
            SmartDashboard.putNumber( "IMU_TotalYaw", ahrs.getAngle() );

            getAngleRotated();

            areGearFlapsClosed();

            isGearOnPeg();

            sleep( 100 );
         }
      }

   }

   @SuppressWarnings( "unused" )
   private class TalonEncoderTester
      implements Runnable
   {

      /*
       * (non-Javadoc)
       *
       * @see java.lang.Runnable#run()
       */
      @Override
      public void run()
      {
         while ( true )
         {
            // final long leftTalonCount = leftCANEncoder.getEncPosition();
            // final long rightTalonCount = rightCANEncoder.getEncPosition();
            // final long leftEncoderCount = -leftDriveEncoder.get();
            // final long rightEncoderCount = -rightDriveEncoder.get();
            // System.out.println( "left: CANTalon=" + leftTalonCount + ", old="
            // + leftEncoderCount );
            // System.out.println( "right: CANTalon=" + rightTalonCount + ",
            // old="
            // + rightEncoderCount );
            final int leftCodeCount = getLeftDriveCount();
            final int rightCodeCount = getRightDriveCount();
            final int averageCount =
               getAverageDriveCount( leftCodeCount, rightCodeCount );
            final double distance = getDistanceDriven();
            System.out.println(
               "distance: left=" + leftCodeCount + ", right=" + rightCodeCount
                  + ", avg=" + averageCount + ", distance=" + distance );

            sleep( 100 );
         }
      }

   }

   private class AutonomousVisionLogger
      implements Runnable
   {

      /*
       * (non-Javadoc)
       *
       * @see java.lang.Runnable#run()
       */
      @Override
      public void run()
      {
         while ( !isEnabled() && !isAutonomous() )
         {
            logger.debug(
               "matchTime={}, distanceDriven={}, newValidCount={}, newAngle={}",
               DriverStation.getInstance().getMatchTime(), getDistanceDriven(),
               vision.newValidCount, vision.newAngle );

            sleep( 100 );
         }
      }

   }

   private class PdpCurrentMonitor
      implements Runnable
   {

      /*
       * (non-Javadoc)
       *
       * @see java.lang.Runnable#run()
       */
      @Override
      public void run()
      {
         final double warnLimit = 200;

         long loopCount = 0;
         while ( true )
         {
            if ( isEnabled() && isOperatorControl() )
            {
               final double pdpTotalCurrent = pdp.getTotalCurrent();
               SmartDashboard.putNumber( "pdpTotalCurrent", pdpTotalCurrent );
               final double pdpDriveCurrent =
                  pdp.getCurrent( 0 ) + pdp.getCurrent( 1 )
                     + pdp.getCurrent( 15 ) + pdp.getCurrent( 2 )
                     + pdp.getCurrent( 3 ) + pdp.getCurrent( 12 );
               SmartDashboard.putNumber( "pdpDriveCurrent", pdpDriveCurrent );
               SmartDashboard.putBoolean( "pdpCurrentWarn",
                  ( pdpDriveCurrent > warnLimit ) );
               if ( ++loopCount > 5 )
               {
                  // logger.debug( "current pdpTotal={}, pdpDrive={}",
                  // pdpTotalCurrent, pdpDriveCurrent );
                  loopCount = 0;
               }
            }

            sleep( 100 );
         }
      }

   }

   private class CameraUpdateListener
      implements ITableListener
   {

      private double angle;
      @SuppressWarnings( "unused" )
      private double validCount;

      private double newAngle;
      private double newValidCount;

      private boolean hasUpdated;

      @SuppressWarnings( "unused" )
      private long fetchCount;


      public CameraUpdateListener()
      {
         angle = 0.0;
         validCount = 0;

         newAngle = 0.0;
         newValidCount = 0;

         hasUpdated = false;
         fetchCount = 0;
      }


      public boolean hasUpdate()
      {
         return hasUpdated;
      }


      public double getAngle()
      {
         fetchCount++;
         // logger.trace( "getAngle = {} fetchCount = {}", angle, fetchCount
         // );
         hasUpdated = false;
         return angle;
      }


      /*
       * (non-Javadoc)
       *
       * @see edu.wpi.first.wpilibj.tables.ITableListener#valueChanged(edu.wpi.
       * first. wpilibj.tables.ITable, java.lang.String, java.lang.Object,
       * boolean)
       */
      @Override
      public void valueChanged( ITable source, String key, Object value,
         boolean isNew )
      {
         try
         {
            switch ( key )
            {
            case "angle":
               // Only fires when the value changes; hence the validCount
               // key
               newAngle = ( (Double) value ).doubleValue();
               // logger.trace( "update angle = {}", format( newAngle, 2 )
               // );
               break;
            case "validCount":
               newValidCount = ( (Double) value ).doubleValue();
               // logger.trace( "update valids = {}", format(
               // newValidCount, 0 )
               // );

               angle = newAngle;
               validCount = newValidCount;
               hasUpdated = true;
               break;
            default:
               logger.warn( "unknown key: {}", key );
               break;
            }
         }
         catch ( final Exception ex )
         {
            logger.error( "got exception: {}", ex.getMessage() );
         }
      }

   }

   private class CameraUpdateListener1
      implements ITableListener
   {

      private double angle;
      @SuppressWarnings( "unused" )
      private double validCount;

      private double newAngle;
      private double newValidCount;

      private boolean hasUpdated;

      @SuppressWarnings( "unused" )
      private long fetchCount;


      public void CameraUpdateListener()
      {
         angle = 0.0;
         validCount = 0;

         newAngle = 0.0;
         newValidCount = 0;

         hasUpdated = false;
         fetchCount = 0;
      }


      public boolean hasUpdate()
      {
         return hasUpdated;
      }


      public double getAngle()
      {
         fetchCount++;
         // logger.trace( "getAngle = {} fetchCount = {}", angle, fetchCount
         // );
         hasUpdated = false;
         return angle;
      }


      /*
       * (non-Javadoc)
       *
       * @see edu.wpi.first.wpilibj.tables.ITableListener#valueChanged(edu.wpi.
       * first. wpilibj.tables.ITable, java.lang.String, java.lang.Object,
       * boolean)
       */
      @Override
      public void valueChanged( ITable source, String key, Object value,
         boolean isNew )
      {
         try
         {
            switch ( key )
            {
            case "angle":
               // Only fires when the value changes; hence the validCount
               // key
               newAngle = ( (Double) value ).doubleValue();
               // logger.trace( "update angle = {}", format( newAngle, 2 )
               // );
               break;
            case "validCount":
               newValidCount = ( (Double) value ).doubleValue();
               // logger.trace( "update valids = {}", format(
               // newValidCount, 0 )
               // );

               angle = newAngle;
               validCount = newValidCount;
               hasUpdated = true;
               break;
            default:
               logger.warn( "unknown key: {}", key );
               break;
            }
         }
         catch ( final Exception ex )
         {
            logger.error( "got exception: {}", ex.getMessage() );
         }
      }

   }

   /** Slop in input values from axis **/
   private static final double zeroInputValue = 0.05;


   private boolean isZero( double inputValue )
   {
      return ( Math.abs( inputValue ) < zeroInputValue );
   }


   static private String format( double value, int digits )
   {
      final String valueString = Double.toString( value );
      final int dotLoc = valueString.indexOf( '.' );
      final int numDigits = valueString.length() - dotLoc - 1;

      final StringBuffer buf = new StringBuffer( valueString );
      if ( numDigits < digits )
      {
         for ( int i = 0; i < ( digits - numDigits ); i++ )
         {
            buf.append( "0" );
         }
      }
      else if ( numDigits == digits )
      {
         // lucky conversion
      }
      else
      {
         buf.setLength( ( dotLoc + digits + 1 ) );
      }

      if ( value > 0 )
      {
         buf.insert( 0, '+' );
      }
      if ( ( buf.charAt( 0 ) != '+' ) && ( buf.charAt( 0 ) != '-' ) )
      {
         buf.insert( 0, ' ' );
      }

      return buf.toString();
   }


   /**
    * @param msec
    **/
   private void sleep( long msec )
   {
      try
      {
         Thread.sleep( msec );
      }
      catch ( final InterruptedException e )
      {
         // We'll ignore it
      }
   }

}
