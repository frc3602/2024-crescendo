package frc.team3602.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  private final Spark blinkin = new Spark(0);

  private boolean hasNote;

  public LEDSubsystem(BooleanSupplier hasNote) {
  this.hasNote = hasNote.getAsBoolean();
  }

  private void setColor(double colorValue){
    blinkin.set(colorValue);
  }

  public Command setLED() {
    var alliance = DriverStation.getAlliance();

    return run(() -> {
      if (hasNote) {
        setColor(0.77);
      } else {
        if (alliance.isPresent()) {
          if (alliance.get() == DriverStation.Alliance.Red) {
            setColor(0.61);
          } else if (alliance.get() == DriverStation.Alliance.Blue) {
            setColor(0.87);
          }
        }
      }
    });
  }
}
