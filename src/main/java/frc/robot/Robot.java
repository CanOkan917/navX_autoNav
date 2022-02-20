/**
 * @author CanOkan, Team3390
 * 
 * Created at 20/02/2022
 */

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
  
  private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(15);
  private final WPI_TalonSRX left2 = new WPI_TalonSRX(14);
  private final WPI_TalonSRX left3 = new WPI_TalonSRX(13);

  private final WPI_TalonSRX rightLeader = new WPI_TalonSRX(0);
  private final WPI_TalonSRX right2 = new WPI_TalonSRX(1);
  private final WPI_TalonSRX right3 = new WPI_TalonSRX(2);

  private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);

  private final AHRS ahrs = new AHRS(Port.kMXP);
  private final PIDController PID = new PIDController(0.03, 0, 0, 0);

  private final Joystick joy = new Joystick(0);

  @Override
  public void robotInit() {
    left2.follow(leftLeader);
    left3.follow(leftLeader);

    right2.follow(rightLeader);
    right3.follow(rightLeader);

    ahrs.calibrate();
    PID.enableContinuousInput(-180, 180);

    // I think its degrees
    PID.setTolerance(1);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    PID.setSetpoint(ahrs.getAngle());
  }

  @Override
  public void autonomousPeriodic() {
    if (!PID.atSetpoint()) {
      double rotation = MathUtil.clamp(PID.calculate(ahrs.getAngle()), -0.7, 0.7);
      drive.arcadeDrive(0, rotation);
    } else {
      drive.arcadeDrive(0.2, 0);
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(joy.getX(), joy.getY());
    if (joy.getRawButtonPressed(1)) {
      autonomousPeriodic();
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
