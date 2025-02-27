package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;

public class APACButtonBox {
  private final Joystick buttonBoxOne = new Joystick(2);
  private final Joystick buttonBoxTwo = new Joystick(3);

  private final ArrayList<Trigger> triggerArray = new ArrayList<Trigger>();

  public APACButtonBox() {
    for (int i = 1; i <= 21; i++) {
      Trigger buttonTrigger =
          new Trigger(
              buttonBoxOne.button(i, CommandScheduler.getInstance().getDefaultButtonLoop()));
      triggerArray.add(buttonTrigger);
    }
    for (int i = 1; i <= 21; i++) {
      Trigger buttonTrigger =
          new Trigger(
              buttonBoxTwo.button(i, CommandScheduler.getInstance().getDefaultButtonLoop()));
      triggerArray.add(buttonTrigger);
    }
  }

  public Trigger reefOneTrigger() {
    return triggerArray.get(6);
  }

  public Trigger reefTwoTrigger() {
    return triggerArray.get(5);
  }

  public Trigger reefThreeTrigger() {
    return triggerArray.get(4);
  }

  public Trigger reefFourTrigger() {
    return triggerArray.get(3);
  }

  public Trigger reefFiveTrigger() {
    return triggerArray.get(2);
  }

  public Trigger reefSixTrigger() {
    return triggerArray.get(1);
  }

  public Trigger reefSevenTrigger() {
    return triggerArray.get(0);
  }

  public Trigger reefEightTrigger() {
    return triggerArray.get(24);
  }

  public Trigger reefNineTrigger() {
    return triggerArray.get(23);
  }

  public Trigger reefTenTrigger() {
    return triggerArray.get(22);
  }

  public Trigger reefElevenTrigger() {
    return triggerArray.get(21);
  }

  public Trigger reefTwelveTrigger() {
    return triggerArray.get(7);
  }

  public Trigger levelOneTrigger() {
    return triggerArray.get(28); // TODO get l1 button working
  }

  public Trigger levelTwoTrigger() {
    return triggerArray.get(27);
  }

  public Trigger levelThreeTrigger() {
    return triggerArray.get(26);
  }

  public Trigger levelFourTrigger() {
    return triggerArray.get(25);
  }
}
