package frc.team3602.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TestPickup extends SequentialCommandGroup {
  public TestPickup() {
    super();

    addCommands(
        new PrintCommand("Test #1"),
        new WaitCommand(0.50),
        new PrintCommand("Test #2"));
  }
}
