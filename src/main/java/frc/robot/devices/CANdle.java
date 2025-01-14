// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.devices;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import frc.robot.Constants;

public class CANdle {

  private final com.ctre.phoenix.led.CANdle candle = new com.ctre.phoenix.led.CANdle(Constants.DeviceConstants.CANDLE_ID);
  private final int LedCount = 60;
  private Animation toAnimate = null;

  private AnimationTypes currentAnimation;

  private GenericEntry ledToggleEntry;
  private GenericEntry animationChooserEntry;

  private boolean ledOn = false;

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll
  }

  public CANdle() {
    configureCANdle();
    changeAnimation(AnimationTypes.SetAll);
  }

  private void configureCANdle() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(configAll, 100);
  }

  public void incrementAnimation() {
    switch (currentAnimation) {
      case ColorFlow -> changeAnimation(AnimationTypes.Fire);
      case Fire -> changeAnimation(AnimationTypes.Larson);
      case Larson -> changeAnimation(AnimationTypes.Rainbow);
      case Rainbow -> changeAnimation(AnimationTypes.RgbFade);
      case RgbFade -> changeAnimation(AnimationTypes.SingleFade);
      case SingleFade -> changeAnimation(AnimationTypes.Strobe);
      case Strobe -> changeAnimation(AnimationTypes.Twinkle);
      case Twinkle -> changeAnimation(AnimationTypes.TwinkleOff);
      case TwinkleOff -> changeAnimation(AnimationTypes.ColorFlow);
      case SetAll -> changeAnimation(AnimationTypes.ColorFlow);
    }
  }

  public final void changeAnimation(AnimationTypes toChange) {
    currentAnimation = toChange;
    switch (toChange) {
      case ColorFlow -> toAnimate = new ColorFlowAnimation(255, 20, 0, 0, 0.7, LedCount, Direction.Forward);
      case Fire -> toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
      case Larson -> toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
      case Rainbow -> toAnimate = new RainbowAnimation(1, 0.1, LedCount);
      case RgbFade -> toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
      case SingleFade -> toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
      case Strobe -> toAnimate = new StrobeAnimation(255, 20, 0, 0, 98.0 / 256.0, LedCount);
      case Twinkle -> toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
      case TwinkleOff -> toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
      case SetAll -> toAnimate = null;
    }
  }

  public void periodic() {
    if (toAnimate != null) {
      candle.animate(toAnimate);
    }
  }
  
  public double getVbat() {
    return candle.getBusVoltage();
  }

  public double get5V() {
    return candle.get5VRailVoltage();
  }
  
  public double getCurrent() {
    return candle.getCurrent();
  }
  
  public double getTemperature() {
    return candle.getTemperature();
  }
  
  public void configBrightness(double percent) {
    candle.configBrightnessScalar(percent, 0);
  }
  
  public void configLedType(LEDStripType type) {
    candle.configLEDType(type, 0);
  }
  
  public void configStatusLedBehavior(boolean offWhenActive) {
    candle.configStatusLedState(offWhenActive, 0);
  }

  public void setLEDColor(int r, int g, int b) {
    candle.setLEDs(r, g, b);
  }

  public void toggleLED() {
    if (ledOn) {
        setLEDColor(0, 0, 0); // Turn off
    } else {
        setLEDColor(115, 21, 191); // Purple (r: 115, g: 21, b: 191)
    }
    ledOn = !ledOn;
  }

  public void initializeShuffleboard() {
    ledToggleEntry = Shuffleboard.getTab("LED Control")
      .add("LED Toggle", false)
      .withWidget("Toggle Button")
      .getEntry();
    animationChooserEntry = Shuffleboard.getTab("LED Control")
      .add("Animation Selector", "ColorFlow")
      .withWidget("ComboBoxChooser")
      .getEntry();
  }

  public void updateLEDStateFromShuffleboard() {
    boolean toggle = ledToggleEntry.getBoolean(false);
    String selectedAnimation = animationChooserEntry.getString("ColorFlow");
    if (toggle) {
      changeAnimation(AnimationTypes.valueOf(selectedAnimation));
    } else {
      setLEDColor(0, 0, 0); // Turn off
    }
  }
}
