package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {
  private static final String kFirstAuto = "2 ball low high";
  private static final String kSecondAuto = "1 ball low delay move";
  private static final String kThirdAuto = "test intake";
  private static final String kFourthAuto = "test shooter";
  private static final String kFifthAuto = "2 ball high";
  private static final String kSixthAuto = "1 ball low";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public String m_autoSelected;

  // public static final double wheelDiameter = 6;
  // public static final double botWidth = 21.1875;

  public static final double intakeSpeedOne = 0.9;

  public double currentRPM = 0.0;
  public double targetRPM = 2800.0;

  public static boolean m_LimelightHasValidTarget = false;
  public static double m_LimelightDriveCommand = 0.0;
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
  public double errAdd = 0;

  public CANSparkMax motorLB;
  public CANSparkMax motorLF;
  public CANSparkMax motorRB;
  public CANSparkMax motorRF;
  public CANSparkMax motorCL;
  public CANSparkMax motorCR;
  public RelativeEncoder encoderCL;
  public RelativeEncoder encoderCR;
  public RelativeEncoder encoderS;
  public MotorControllerGroup motorsL;
  public MotorControllerGroup motorsR;
  public double aRot;
  public double aFwd;

  public CANSparkMax motorI;
  public CANSparkMax motorS;
  public Joystick joystickMain;
  public Joystick joystickAlt;
  public JoystickButton mainButt1;
  public JoystickButton mainButt2;
  public JoystickButton altButt1;
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
  public boolean tankFlag = false;

  public double autoDelay = 3;
  public Timer mainTimer;

  @Override
  public void robotInit() {
    m_chooser.addOption(kFirstAuto, kFirstAuto);
    m_chooser.addOption(kSecondAuto, kSecondAuto);
    m_chooser.addOption(kThirdAuto, kThirdAuto);
    m_chooser.addOption(kFourthAuto, kFourthAuto);
    m_chooser.setDefaultOption(kFifthAuto, kFifthAuto);
    m_chooser.addOption(kSixthAuto, kSixthAuto);
    SmartDashboard.putData("Autos", m_chooser);

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

    motorI = new CANSparkMax(12, MotorType.kBrushless);
    motorS = new CANSparkMax(13, MotorType.kBrushless);
    motorS.setInverted(true);
    motorS.setSmartCurrentLimit(50);
    encoderS = motorS.getEncoder();
    joystickMain = new Joystick(0);
    joystickAlt = new Joystick(2);
    mainButt1 = new JoystickButton(joystickMain, 1);
    mainButt2 = new JoystickButton(joystickMain, 2);
    altButt1 = new JoystickButton(joystickAlt, 1);
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
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    mainTimer.reset();
    mainTimer.start();
  }
  // AUTO
  @Override
  public void autonomousPeriodic() {
    currentRPM = encoderS.getVelocity();
    driveTrain.feed();
    double currTime = mainTimer.get();
    switch (m_autoSelected) {
      case kFirstAuto:
        if (currTime < 15) {
          if (currTime > 4.5 && currTime < 7) {
            driveTrain.arcadeDrive(-0.6, 0.15);
          } else {
            driveTrain.stopMotor();
          }
          if (currTime > 7.3) {
            targetRPM = 5000;
          } else {
            targetRPM = 2500;
          }
          if (currTime < 2.75 || (currTime > 7.3 && currTime < 10.05)) {
            motorS.set(calcShooterRPM());
          } else {
            motorS.stopMotor();
          }
          if (((currentRPM < targetRPM+150 && currentRPM > targetRPM-250)||motorI.get()!=0) && currTime < 2.75) {
            motorI.set(-intakeSpeedOne);
          } else if (currTime > 6.6 && currTime < 7.3) {
            motorI.set(-intakeSpeedOne);
          } else if (((currentRPM < targetRPM+300 && currentRPM > targetRPM-300)||motorI.get()!=0) && currTime > 7.3) {
            motorI.set(-intakeSpeedOne);
          } else {
            motorI.stopMotor();
          }
          if (currTime > 10.05) {
            motorI.stopMotor();
          }
        }
        break;
      case kSecondAuto:
      targetRPM = 2500;
        if (currTime < 15) {
          if (currTime < 2.5) {
            motorS.set(calcShooterRPM());
            if ((currentRPM < targetRPM+150 && currentRPM > targetRPM-250) || motorI.get()!=0) {
              motorI.set(-intakeSpeedOne);
            }
          } else if (currTime > 2.5) {
            if (currTime > 8 && currTime < 10.5) {
              driveTrain.arcadeDrive(-0.6, 0.15);
            } else {
              driveTrain.stopMotor();
            }
            motorS.stopMotor();
            motorI.stopMotor();
          }
        }
        break;
      case kThirdAuto:
        if (currTime < 1) {
          motorI.set(-0.5);
        } else {
          motorI.stopMotor();
        }
        break;
      case kFourthAuto:
        if (currTime < 1) {
          targetRPM = 2000;
          motorS.set(calcShooterRPM());
        } else {
          motorS.stopMotor();
        }
        break;
      case kFifthAuto:
        targetRPM = 5000;
        if (currTime < 15) {
          if (currTime < 2.5) {
            driveTrain.arcadeDrive(-0.6, 0.15);
          } else {
            driveTrain.stopMotor();
          }
          if (currTime > 2.8 && currTime < 5.55) {
            motorS.set(calcShooterRPM());
          } else {
            motorS.stopMotor();
          }
          if (((currentRPM < targetRPM+300 && currentRPM > targetRPM-300)||motorI.get()!=0) && currTime > 2.8) {
            motorI.set(-intakeSpeedOne);
          } else {
            motorI.stopMotor();
          }
          if (currTime > 5.55) {
            motorI.stopMotor();
          }
        }
        break;
      case kSixthAuto:
        targetRPM = 2500;
        if (currTime < 15) {
          if (currTime < 2.5) {
            motorS.set(calcShooterRPM());
            if ((currentRPM < targetRPM+150 && currentRPM > targetRPM-250) || motorI.get()!=0) {
              motorI.set(-intakeSpeedOne);
            }
          } else {
            motorI.stopMotor();
            motorS.stopMotor();
          }
        }
        break;
      default:
        break;
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    currentRPM = encoderS.getVelocity();
    // intake
    // lt and rt (left and right triggers -> buttons 7 and 8)
    if (joystickMain.getRawButton(1)) {
      motorI.set(intakeSpeedOne);
    } else if (joystickMain.getRawButton(2)) {
      motorI.set(-intakeSpeedOne);
    } else {
      motorI.stopMotor();
     }
     if (ctlrButt7.get() || altButt1.get() || mainButt2.get()) {
      rampMotor(motorI, intakeSpeedOne, rampSpeedIntake);
    } else if (ctlrButt8.get() || mainButt1.get()) {
       rampMotor(motorI, -intakeSpeedOne, rampSpeedIntake);
     } else {
       rampMotor(motorI, 0, rampSpeedIntake);
    }
  
    // Update_Limelight_Tracking();

    // drivetrain
    double drive = joystickMain.getY();
    double steering = -joystickMain.getX();
    if (joystickMain.getRawButtonPressed(11)) reverseFlag = reverseFlag ? false : true;
    SmartDashboard.putBoolean("Reverse", reverseFlag);
    driveTrain.arcadeDrive(drive*(reverseFlag?-1:1), steering);
    



    // if (ctlrButt9.get()) {
    //   drive = m_LimelightDriveCommand;
    //   steering = m_LimelightSteerCommand;
    //   driveTrain.arcadeDrive(drive, steering);
    // } else {
    //   //if (joystickMain.getRawButtonPressed(7)) tankFlag = tankFlag ? false : true;
    //   if (joystickMain.getRawButtonPressed(11)) reverseFlag = reverseFlag ? false : true;
    //   // SmartDashboard.putBoolean("Tank Drive", tankFlag);
    //   SmartDashboard.putBoolean("Reverse", reverseFlag);
    //   if (tankFlag) {
    //     if (reverseFlag) driveTrain.tankDrive(-joystickMain.getY(), -joystickAlt.getY());
    //     else driveTrain.tankDrive(joystickAlt.getY(), joystickMain.getY());
    //   }
    //   else driveTrain.arcadeDrive(drive*(reverseFlag?-1:1),steering);
    // }

// double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    // shooter
    if (joystickMain.getRawButton(3)) {
      targetRPM = 2800;
      motorS.set(calcShooterRPM());
    }
     if (controllerMain.getPOV()!=-1) {
      handleDPad(controllerMain.getPOV());
     } else if(ctlrButt10.get()){
       calculateVelocity(angleToDistanceLime(ty), 0, minDistance);
     } else {
       motorS.stopMotor();
     }
    
     if(controllerMain.getRawButtonPressed(5)) errAdd += 0.01;
     else if(controllerMain.getRawButtonPressed(11)) errAdd -= 0.01;
    
    SmartDashboard.putNumber("Error Val", 4.5+errAdd);

     if (ctlrButt1.get()) {
       motorCR.set(0.4);
     } else if (ctlrButt3.get()) {
       motorCL.set(-0.4);
     }else{
       motorCL.stopMotor();
       motorCR.stopMotor();
     }

     if (controllerMain.getRawButtonPressed(12)) {
       encoderCL.setPosition(0);
       encoderCR.setPosition(0);
     }

     boolean testingClimb = true;

    // climb
     //y = up=4  a = down=2
     double positionCL = encoderCL.getPosition();
     double positionCR = encoderCL.getPosition();
     if (ctlrButt4.get()) {
       if (positionCL > upperLimitL-10) {
         if (positionCL > upperLimitL-5) {
          motorCL.stopMotor();
        } else motorCL.set(0.2);
      } else {
        motorCL.set(testingClimb?0.5:1);
      }
      if (positionCR > upperLimitR-10) {
        if (positionCR > upperLimitR-5) {
          motorCR.stopMotor();
        } else motorCR.set(-0.2);
      } else {
        motorCR.set(testingClimb?-0.5:-1);
      }
    } else if (ctlrButt2.get()) {
      if (positionCL < lowerLimitL+30) {
        if (positionCL < lowerLimitL+25) {
          motorCL.stopMotor();
        } else motorCL.set(-0.2);
      } else {
        motorCL.set(testingClimb?-0.3:-0.7);
      }
      if (positionCR < lowerLimitR+30) {
        if (positionCR < lowerLimitR+25) {
          motorCR.stopMotor();
        } else motorCR.set(0.2);
      } else {
        motorCR.set(testingClimb?0.3:0.7);
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
    // These numbers must be tuned for your Robot!  Be careful!
    final double STEER_K = 0.02;                    // how hard to turn toward the target
    final double STEER_MIN = 0.3;
    final double DRIVE_K = 0.2;                    // how hard to drive fwd toward the target
    final double MAX_DRIVE = 0.5;                   // Simple speed limit so we don't drive too fast

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    if (tv < 1.0){
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    m_LimelightHasValidTarget = true;

    // Start with proportional steering
    double steer_cmd = -tx * STEER_K;
    if(steer_cmd > 0) steer_cmd += STEER_MIN;
    else if(steer_cmd < 0) steer_cmd -= STEER_MIN;
    m_LimelightSteerCommand =  steer_cmd;

    // try to drive forward until the target area reaches our desired area
   //(DESIRED_TARGET_AREA - ta) * DRIVE_K
    
    double distance = angleToDistanceLime(ty);
    double drive_cmd = (110-distance)*DRIVE_K;
    
    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE)
    {
      drive_cmd = MAX_DRIVE;
    }else if(drive_cmd < -MAX_DRIVE){
      drive_cmd = -MAX_DRIVE;
    }
    double m_LimelightDriveCommand = -drive_cmd;
    //System.out.println(distance);
    
  }

//calculates velocity for shooter
// public void calculateVelocity(double dist, double vert, double maxMinDist){

//   dist = (dist+36)/39.37;

//     if(((dist >= maxMinDist && vert == heightH)||(dist <= maxMinDist && vert == heightL))) rampMotor(motorS, 0, rampSpeedShooter);
//     else{

//       //multiply by error
//       velocity = Math.sqrt((-9.81*Math.pow(dist, 2))/((2*Math.pow(Math.cos(angle), 2))*(vert - (((Math.sin(angle)*dist))/(Math.cos(angle))))));
//       velocityAng = velocity/radius;
//       targetRPM = ((4.5+errAdd)*velocityAng*60)/(2*Math.PI);

//       currentRPM = encoderS.getVelocity();
//       System.out.println(dist);
//       System.out.println(targetRPM);

//       motorS.set(calcShooterRPM());
//     }

//     if(vert == 0){
//       int testDist = 1;
//       //multiply by error
//       velocity = Math.sqrt((-9.81*Math.pow(testDist, 2))/((2*Math.pow(Math.cos(angle), 2))*(vert - (((Math.sin(angle)*testDist))/(Math.cos(angle))))));
//       System.out.println(velocity);
//       velocityAng = velocity/radius;
//       targetRPM = ((4.5+errAdd)*velocityAng*60)/(2*Math.PI);

//       currentRPM = encoderS.getVelocity();

//       System.out.println(targetRPM);

//       motorS.set(Math.abs(calcShooterRPM()));
//     }
// }

  // public void rampMotor(CANSparkMax motor, double speedWanted, double rampSpeed) {
  //   if (speedWanted == 0) {
  //     motor.stopMotor();
  //   }
  //   else {
  //     double currSpeed = motor.get();
  //     double speedError = speedWanted-currSpeed;
  //     if (speedError == 0) {
  //       return;
  //     } else if (speedError > 0 && currSpeed < speedWanted) {
  //       motor.set(currSpeed+rampSpeed);
  //     } else if (speedError < 0 && currSpeed > speedWanted) {
  //       motor.set(currSpeed-rampSpeed);
  //     }
  //   }
  // }

  // dpad dealing with input
  public void handleDPad(int angle) {
    // dpad up = increase rpm by 20
    // dpad down = decrease rpm by 20
    // dpad right = run flywheel

    if (targetRPM < 0) {
      targetRPM = 0;
    }

    currentRPM = encoderS.getVelocity();

    switch (angle) {
      case 0: // up
        targetRPM += 20;
        break;
      case 90: // right
        motorS.set(calcShooterRPM());
        break;
      case 180: // down
        targetRPM -= 20;
        break;
      default: // any other direction
        break;
      }
    //print to smart dashboard 
    SmartDashboard.putNumber("Current RPMs", currentRPM);
    SmartDashboard.putNumber("Target RPMs", targetRPM);
  }

  public double calcShooterRPM() {
    currentRPM = encoderS.getVelocity();
    double val = targetRPM / 5800;
    return val > 1 ? 1 : val;
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