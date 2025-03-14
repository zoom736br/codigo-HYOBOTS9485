package frc.robot.joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controle;

public class DriverController {

  private static DriverController mInstance = null;

  public static DriverController getInstance() {
    if (mInstance == null) {
      mInstance = new DriverController();
    }

    return mInstance;
  }

  final CommandXboxController driverController;

  private double allianceInputDirectionCorrection = 1;

  private DriverController() {
    driverController = new CommandXboxController(Controle.xboxControle);
  }

  public void changeAlliances() {
    allianceInputDirectionCorrection = allianceInputDirectionCorrection * -1;
  }

  public double getXtranslation() {
    if (turboActivate().getAsBoolean()) {
      return -MathUtil.applyDeadband(performAllianceInputDirectionCorrection(driverController.getLeftX()),
          Constants.Controle.DEADBAND) * allianceInputDirectionCorrection;
    }
    return -MathUtil.applyDeadband(performAllianceInputDirectionCorrection(driverController.getLeftX()),
        Constants.Controle.DEADBAND) * 0.6 * allianceInputDirectionCorrection;
  }

  public double getYtranslation() {
    if (turboActivate().getAsBoolean()) {
      return -MathUtil.applyDeadband(performAllianceInputDirectionCorrection(driverController.getLeftY()),
          Constants.Controle.DEADBAND) * allianceInputDirectionCorrection;
    }
    return -MathUtil.applyDeadband(performAllianceInputDirectionCorrection(driverController.getLeftY()),
        Constants.Controle.DEADBAND) * 0.6 * allianceInputDirectionCorrection;
  }

  public double getRotation() {
    return MathUtil.applyDeadband(driverController.getRightX(), Constants.Controle.DEADBAND);
  }
  
  public Trigger turboActivate() {
    return driverController.rightTrigger(0.2);
  }

  public Trigger a() {
    return driverController.a();
  }

  public Trigger y() {
    return driverController.y();
  }

  public Trigger x() {
    return driverController.x();
  }

  public Trigger b() {
    return driverController.b();
  }

  public Trigger leftBumper() {
    return driverController.leftBumper();
  }

  public Trigger rightBumper() {
    return driverController.rightBumper();
  }

  private double performAllianceInputDirectionCorrection(Double value) {
    Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get()
        : DriverStation.Alliance.Red;
    if (alliance == Alliance.Red) {
      return -value;
    }
    return value;
  }
}
