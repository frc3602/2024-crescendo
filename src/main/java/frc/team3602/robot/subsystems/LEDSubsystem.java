/* package frc.team3602.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private final double kGreen = 0.77;
  private final double kBlue = 0.87;
  private final double kRed = 0.61;

  public boolean isBlueAlliance, isRedAlliance;

  private final Spark blinkin = new Spark(0);

  public LEDSubsystem() {
  }

  @Override
  public void periodic() {
    isBlueAlliance = isBlueAlliance();
    isRedAlliance = isRedAlliance();
  }

  private void setColor(double colorValue) {
    blinkin.set(colorValue);
  }

  public Command setGreen() {
    return runOnce(() -> {
      setColor(kGreen);
    });
  }

  public Command setBlue() {
    return runOnce(() -> {
      setColor(kBlue);
    });
  }

  public Command setRed() {
    return runOnce(() -> {
      setColor(kRed);
    });
  }

  public Command setAlliance() {
    return runOnce(() -> {
      if (isBlueAlliance) {
        setColor(kBlue);
      } else if (isRedAlliance) {
        setColor(kRed);
      }
    });
  }

  public boolean isBlueAlliance() {
    var alliance = DriverStation.getAlliance();

    return alliance.get() == DriverStation.Alliance.Blue;
  }

  public boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();

    return alliance.get() == DriverStation.Alliance.Red;
  }
}

*/
