package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.EntryListenerFlags;

public class PIDNTValue {
  public PIDNTValue(double kP, double kI, double kD, double kF, TalonFX motor, String name) {
    NTValue kPNTValue = new NTValue(kP, "kP " + name, "PID " + name);
    kPNTValue.entry.addListener((event) -> {
      motor.config_kP(0, event.value.getDouble(), 30);
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    motor.config_kP(0, kPNTValue.value, 30);
    NTValue kINTValue = new NTValue(kI, "kI " + name, "PID " + name);
    kINTValue.entry.addListener((event) -> {
      motor.config_kI(0, event.value.getDouble(), 30);
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    motor.config_kI(0, kINTValue.value, 30);
    NTValue kDNTValue = new NTValue(kD, "kD " + name, "PID " + name);
    kDNTValue.entry.addListener((event) -> {
      motor.config_kD(0, event.value.getDouble(), 30);
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    motor.config_kD(0, kDNTValue.value, 30);
    NTValue kFNTValue = new NTValue(kF, "kF " + name, "PID " + name);
    kFNTValue.entry.addListener((event) -> {
      motor.config_kF(0, event.value.getDouble(), 30);
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    motor.config_kF(0, kFNTValue.value, 30);
  }
  
  public PIDNTValue(double kP, double kI, double kD, PIDController pid, String name) {
    NTValue kPNTValue = new NTValue(kP, "kP " + name, "PID " + name);
    kPNTValue.entry.addListener((event) -> {
      pid.setP(event.value.getDouble());
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    NTValue kINTValue = new NTValue(kI, "kI " + name, "PID " + name);
    kINTValue.entry.addListener((event) -> {
      pid.setI(event.value.getDouble());
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    NTValue kDNTValue = new NTValue(kD, "kD " + name, "PID " + name);
    kDNTValue.entry.addListener((event) -> {
      pid.setD(event.value.getDouble());
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }
}