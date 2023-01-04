// 3000 low bumper touching tower
// 4800 high end of tarmac

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.net.PortForwarder;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // public static final double wheelDiameter = 6;
  // public static final double botWidth = 21.1875;

  public static final double intakeSpeedTwo = 0.8;

  public double currentRPM = 0.0;
  public double targetRPM = 4980.0;
  public double target = 0.5;
  public double closedLoopCommand = 0.0;

  public static final double shooterSpeedOne = 0.25;
  public static final double shooterSpeedTwo = 0.3;
  public static final double shooterSpeedThree = 0.35;
  public static final double shooterSpeedFour = 0.4;

  public static Encoder flyWheelEncoder;

  public static final boolean closedLoop = true; 
  public static final double kS = 0.0;
  public static final double kV = 0.4/5500;
  public static final double kA = 0.0;
  public static final double kP = 0.5/500;

  public static boolean m_LimelightHasValidTarget = false;
  //public static double m_LimelightDriveCommand = 0.0;
  public static double m_LimelightSteerCommand = 0.0;
  public static double radius = 0.0508;
  
  public static final double shooterSpeedFive = 0.5;
  public static final double shooterSpeedSix = 0.6;
  public static final double rampSpeedShooter = 0.05;
  public static final double rampSpeedIntake = 0.05;
  public static final double rotationSpeed = 0.3;
  public static final double threshold = 2;
  public static final double kp = 1;

  public static final double upperLimitL = 465;
  public static final double lowerLimitL = 5;
  public static final double upperLimitR = 420;
  public static final double lowerLimitR = 5;

  public static final double angle = Math.toRadians(42);
  public static final double distanceError = 2; //undetermined
  public static final double minDistance = 110; //undetermined
  public static final double maxDistance = 115;
  public static final double heightH = 1.9304; 
  public static final double heightL = 0.3302; 
  public static final double maxShootSpeed = 0; //undetermined
  public double velocity = 0;
  public double velocityAng = 0;
  public double shooterSpeed = 0;

  public CANSparkMax motorLB;
  public CANSparkMax motorLF;
  public CANSparkMax motorRB;
  public CANSparkMax motorRF;
  public CANSparkMax motorCL;
  public CANSparkMax motorCR;
  public RelativeEncoder encoderCL;
  public RelativeEncoder encoderCR;
  public MotorControllerGroup motorsL;
  public MotorControllerGroup motorsR;
  public RelativeEncoder encoderL;
  public RelativeEncoder encoderR;
  public double aRot;
  public double aFwd;

  public CANSparkMax motorI;
  public CANSparkMax motorS;
  public Joystick joystickMain;
  public JoystickButton butt1;
  public JoystickButton butt11;
  public Joystick controllerMain;
  public JoystickButton ctlrButt1;
  public JoystickButton ctlrButt2;
  public JoystickButton ctlrButt3;
  public JoystickButton ctlrButt4;
  public JoystickButton ctlrButt5;
  public JoystickButton ctlrButt6;
  public JoystickButton ctlrButt7;
  public JoystickButton ctlrButt8;
  public JoystickButton ctlrButt9;
  public JoystickButton ctlrButt10;
  public JoystickButton ctlrButt11;
  public JoystickButton ctlrButt12;
  public POVButton keypad0;
  public POVButton keypad90;
  public POVButton keypad180;
  public POVButton keypad270;
  public DifferentialDrive driveTrain;

  public boolean reverseFlag = false;
  public boolean reverseFlagFlag = false;

  public double autoDelay = 3;
  public Timer mainTimer;

  @Override
  public void robotInit() {
    PortForwarder.add(5800, "10.87.17.11", 5800);
    PortForwarder.add(5800, "10.87.17.11", 5801);
    PortForwarder.add(5800, "10.87.17.11", 5802);
    PortForwarder.add(5800, "10.87.17.11", 5803);
    PortForwarder.add(5800, "10.87.17.11", 5804);
    PortForwarder.add(5800, "10.87.17.11", 5805);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    flyWheelEncoder = new Encoder(1, 2);
    motorLB = new CANSparkMax(4, MotorType.kBrushed);
    motorLF = new CANSparkMax(2, MotorType.kBrushed);
    motorRB = new CANSparkMax(5, MotorType.kBrushed);
    motorRF = new CANSparkMax(3, MotorType.kBrushed);
    motorCL = new CANSparkMax(20, MotorType.kBrushless);
    motorCR = new CANSparkMax(21, MotorType.kBrushless);
    motorCL.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motorCR.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motorCR.setInverted(true);
    encoderCL = motorCL.getEncoder();
    encoderCR = motorCR.getEncoder();
    motorsL = new MotorControllerGroup(motorLB, motorLF);
    motorsR = new MotorControllerGroup(motorRB, motorRF);
    motorsR.setInverted(true);
    encoderL = motorLF.getAlternateEncoder(Type.kQuadrature, 8192);
    encoderL.setInverted(true);
    encoderR = motorRF.getAlternateEncoder(Type.kQuadrature, 8192);

    motorI = new CANSparkMax(12, MotorType.kBrushed);
    motorS = new CANSparkMax(13, MotorType.kBrushed);
    joystickMain = new Joystick(0);
    butt1 = new JoystickButton(joystickMain, 1);
    butt11 = new JoystickButton(joystickMain, 11);
    controllerMain = new Joystick(1);
    ctlrButt1 = new JoystickButton(controllerMain, 1);
    ctlrButt2 = new JoystickButton(controllerMain, 2);
    ctlrButt3 = new JoystickButton(controllerMain, 3);
    ctlrButt4 = new JoystickButton(controllerMain, 4);
    ctlrButt5 = new JoystickButton(controllerMain, 5);
    ctlrButt6 = new JoystickButton(controllerMain, 6);
    ctlrButt7 = new JoystickButton(controllerMain, 7);
    ctlrButt8 = new JoystickButton(controllerMain, 8);
    ctlrButt9 = new JoystickButton(controllerMain, 9);
    ctlrButt10 = new JoystickButton(controllerMain, 10);
    ctlrButt11 = new JoystickButton(controllerMain, 11);
    ctlrButt12 = new JoystickButton(controllerMain, 12);
    driveTrain = new DifferentialDrive(motorsL, motorsR);

    mainTimer = new Timer();
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setFPS(15);
    camera.setResolution(160, 120);
    camera.setExposureManual(3);
    camera.setWhiteBalanceManual(0);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("CurrentRPM", currentRPM);
    SmartDashboard.putNumber("TargetRPM", targetRPM);
    SmartDashboard.putNumber("Target", target);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    //m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);
    mainTimer.reset();
    mainTimer.start();
  }
  // AUTO
  @Override
  public void autonomousPeriodic() {
    currentRPM = 60*flyWheelEncoder.getRate()/2048;
    targetRPM = 3000;
    driveTrain.feed();
    double currTime = mainTimer.get();
    if (currTime < 15) {
      if (currTime > 4.5 && currTime < 8) {
        driveTrain.arcadeDrive(-0.6, 0.225);
      } else {
        driveTrain.stopMotor();
      }
      if (currTime > 8.3) {
        targetRPM = 4900;
      }
      if (currTime < 2.75 || (currTime > 8.3 && currTime < 11.05)) {
        motorS.set(calcShooterRPM());
      } else {
        motorS.stopMotor();
      }
      if (((currentRPM < targetRPM+150 && currentRPM > targetRPM-250)||motorI.get()!=0) && currTime < 2.75) {
        rampMotor(motorI, intakeSpeedTwo, rampSpeedIntake);
      } else if (currTime > 6.8 && currTime < 8.3) {
        rampMotor(motorI, intakeSpeedTwo, rampSpeedIntake);
      } else if (((currentRPM < targetRPM+300 && currentRPM > targetRPM-300)||motorI.get()!=0) && currTime > 8.3) {
        rampMotor(motorI, intakeSpeedTwo, rampSpeedIntake);
      } else {
        motorI.stopMotor();
      }
    }
    /*switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        autoRotate(90);
        updateMotors(motorS, shooterSpeedThree, rampSpeedShooter);
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        autoRotate(90);
        updateMotors(motorS, shooterSpeedThree, rampSpeedShooter);
        break;
    }*/
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    // intake
    // lt and rt (left and right triggers -> buttons 7 and 8)
    if (ctlrButt7.get()) {
      rampMotor(motorI, -intakeSpeedTwo, rampSpeedIntake);
    } else if (ctlrButt8.get() || butt1.get()) {
      rampMotor(motorI, intakeSpeedTwo, rampSpeedIntake);
    } else {
      rampMotor(motorI, 0, rampSpeedIntake);
    }
  
    Update_Limelight_Tracking();

    // drivetain
    double drive = joystickMain.getY();
    double steering = -joystickMain.getX();

    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    if (ctlrButt9.get()) {
      steering = m_LimelightSteerCommand;
    }

    // streamlined reverse
    if (joystickMain.getRawButtonPressed(11)) reverseFlag = reverseFlag ? false : true;
    driveTrain.arcadeDrive(drive*(reverseFlag?1:-1),steering);

    // shooter
    if (controllerMain.getPOV()!=-1 && !controllerMain.getRawButton(6)) {
      handleDPad(controllerMain.getPOV());
    }else if(ctlrButt10.get()){
      calculateVelocity(1, 0, 0);
    }else if (ctlrButt6.get()) {
      currentRPM = 60*flyWheelEncoder.getRate()/2048;
      targetRPM = 4980;
      calcShooterRPM();
      motorS.set(closedLoopCommand);
    }else if (ctlrButt5.get()) {
      calculateVelocity(angleToDistanceLime(ty), heightH, minDistance);
    } else {
      motorS.stopMotor();
    }

    
    
    // manually lower the two climbs, left and right motors when looking from the front
    // X button (left) and B button (right) -> buttons 1 and 3
    if (ctlrButt1.get()) {
      motorCR.set(0.6);
    } else if (ctlrButt3.get()) {
      motorCL.set(-0.6);
    } else {
      motorCL.stopMotor();
      motorCR.stopMotor();
    }

    // right thumb thingy
    if (controllerMain.getRawButtonPressed(12)) {
      encoderCL.setPosition(0);
      encoderCR.setPosition(0);
    }

    // climb
    // Y button (up) and A button (down) -> buttons 4 and 2
    double positionCL = encoderCL.getPosition();
    double positionCR = encoderCL.getPosition();
    if (ctlrButt4.get()) {
      if (positionCL > upperLimitL-10) {
        if (positionCL > upperLimitL-5) {
          motorCL.stopMotor();
        } else motorCL.set(0.2);
      } else {
        motorCL.set(1);
      }
      if (positionCR > upperLimitR-10) {
        if (positionCR > upperLimitR-5) {
          motorCR.stopMotor();
        } else motorCR.set(-0.2);
      } else {
        motorCR.set(-1);
      }
    } else if (ctlrButt2.get()) {
      if (positionCL < lowerLimitL+30) {
        if (positionCL < lowerLimitL+25) {
          motorCL.stopMotor();
        } else motorCL.set(-0.2);
      } else {
        motorCL.set(-1);
      }
      if (positionCR < lowerLimitR+30) {
        if (positionCR < lowerLimitR+25) {
          motorCR.stopMotor();
        } else motorCR.set(0.2);
      } else {
        motorCR.set(1);
      }
    } else {
      if (ctlrButt1.get() || ctlrButt3.get()) return;
      motorCL.stopMotor();
      motorCR.stopMotor();
    }
  }

  public double angleToDistanceLime(double tY){
    //128 in = -22.3
    //97 in = -17.5
    //67 in = -11.11
    //38 in = .9
    
    double limelightMountAngleDegrees = 44.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 31.0;

    // distance from the target to the floor
    double goalHeightInches = 102.5;

    double angleToGoalDegrees = limelightMountAngleDegrees + tY;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    return distanceFromLimelightToGoalInches;
  }


  public void Update_Limelight_Tracking(){
        final double STEER_K = 0.02;                    // how hard to turn toward the target
        final double STEER_MIN = 0.3;

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

        if (tv < 1.0){
          m_LimelightHasValidTarget = false;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        double steer_cmd = -tx * STEER_K;
        if(steer_cmd > 0) steer_cmd += STEER_MIN;
        else if(steer_cmd < 0) steer_cmd -= STEER_MIN;
        m_LimelightSteerCommand = steer_cmd;
  }

//calculates velocity for shooter
public void calculateVelocity(double dist, double vert, double maxMinDist){

  dist = (dist+25)/39.37;
  double calculatedRPM = 0;

    if(((dist <= maxMinDist && vert == heightH)||(dist >= maxMinDist && vert == heightL))) motorS.set(0);
    else{

      //multiply by error
      velocity = 4.25*Math.sqrt((-9.81*Math.pow(dist, 2))/((2*Math.pow(Math.cos(angle), 2))*(vert - (((Math.sin(angle)*dist))/(Math.cos(angle))))));
      velocityAng = velocity/radius;
      targetRPM = (velocityAng*60)/(2*Math.PI);

      currentRPM = 60*flyWheelEncoder.getRate()/2048;

      motorS.set(Math.abs(calcShooterRPM()));
    }

    if(vert == 0){
      int testDist = 2;
      //multiply by error
      velocity = 4.25*Math.sqrt((-9.81*Math.pow(testDist, 2))/((2*Math.pow(Math.cos(angle), 2))*(vert - (((Math.sin(angle)*testDist))/(Math.cos(angle))))));
      System.out.println(velocity);
      velocityAng = velocity/radius;
      targetRPM = (velocityAng*60)/(2*Math.PI);

      currentRPM = 60*flyWheelEncoder.getRate()/2048;

      System.out.println(targetRPM);
      System.out.println(closedLoopCommand);

      motorS.set(Math.abs(calcShooterRPM()));
    }
}


  public void rampMotor(CANSparkMax motor, double speedWanted, double rampSpeed) {
    if (speedWanted == 0) {
      motor.stopMotor();
    }
    else {
      double currSpeed = motor.get();
      double speedError = speedWanted-currSpeed;
      if (speedError == 0) {
        return;
      } else if (speedError > 0 && currSpeed < speedWanted) {
        motor.set(currSpeed+rampSpeed);
      } else if (speedError < 0 && currSpeed > speedWanted) {
        motor.set(currSpeed-rampSpeed);
      }
    }
  }

  // dpad dealing with input
  public void handleDPad(int angle) {
    // dpad up = increase rpm by 20
    // dpad down = decrease rpm by 20
    // dpad right = run flywheel

    if (targetRPM < 0) {
      targetRPM = 0;
    }

    currentRPM = 60*flyWheelEncoder.getRate()/2048;

    switch (angle) {
      case 0: // up
        targetRPM += 20;
        break;
      case 90: // right
        calcShooterRPM();
        if (closedLoop) {
          motorS.set(closedLoopCommand);
        } else { 
          rampMotor(motorS, target, rampSpeedShooter);
        }
        break;
      case 180: // down
        targetRPM -= 20;
        break;
      default: // any other direction
        break;
      }
    //print to smart dashboard 

  }

  public double calcShooterRPM() {
    double RPMerror = targetRPM - currentRPM;
    closedLoopCommand = RPMerror*kP;
    closedLoopCommand += targetRPM*kV;
    closedLoopCommand += kS;
    return closedLoopCommand;
  }
  public double RPMToPercentOutput(double targ, double cur)
  {
    double RPMerror = targ - cur;
    double command = RPMerror*kP;
    command += targetRPM*kV;
    command += kS;
    return closedLoopCommand;
  }

  @Override
  public void disabledInit() {
    driveTrain.stopMotor();
    motorI.stopMotor();
    motorS.stopMotor();
    mainTimer.stop();
    mainTimer.reset();
  }
}