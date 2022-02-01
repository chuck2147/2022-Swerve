package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class NTValue {
  public NTValue(double initialValue, String key) {
    this(initialValue, key, "NTValues");
  }

  public NTValue (double initialValue, String key, String tabName) {
    ShuffleboardTab tab = Shuffleboard.getTab(tabName);
    NetworkTableEntry entry = tab.add(key, initialValue)
      .withSize(2, 1)
      .withWidget(BuiltInWidgets.kTextView) // specify the widget here
      .getEntry();
    entry.setValue(initialValue);
    entry.addListener(event -> {
      value = entry.getValue().getDouble();
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    value = initialValue;
    this.entry = entry;
  }
  public double value;
  public NetworkTableEntry entry;
}

