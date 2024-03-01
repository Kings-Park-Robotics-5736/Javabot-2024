package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class LEDSubsystem extends SubsystemBase {
    private CANdle candle;
    private CANdleConfiguration config;
    private int LedCount = 300;
    
    private Animation colorFlowAnimation = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
    private Animation fireAnimation = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
    private Animation larsonAnimation = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
    private Animation rainbowAnimation = new RainbowAnimation(1, 0.1, LedCount);
    private Animation rgbFadeAnimation = new RgbFadeAnimation(0.7, 0.4, LedCount);
    private Animation singleFadeAnimation= new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
    private Animation strobeAnimation = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
    private Animation twinkleAnimation = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
    private Animation twinkleOffAnimation= new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);


    public LEDSubsystem() {
        candle = new CANdle(LEDConstants.CANdleID, "rio");
        config = new CANdleConfiguration();  
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1.0;
        config.statusLedOffWhenActive = false;
        config.disableWhenLOS = false;
        config.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(config, 100);
    }


    public void setAllLEDColor(int[] rgb) {
        candle.setLEDs(rgb[0], rgb[1], rgb[2]);
        candle.modulateVBatOutput(0.9);
      }
    
    // Turn one LED to a specific color
    public void setOneLEDColor(int[] rgb, int ledNumber) {
    candle.setLEDs(rgb[0], rgb[1], rgb[2], 0, ledNumber, 1);
    candle.modulateVBatOutput(0.9);
    }
    
    public void setLEDOff() {
    candle.setLEDs(0, 0, 0);
    candle.modulateVBatOutput(0);
    }
    

    public void animate(String toAnimate) {
        if (toAnimate == "ColorFlowAnimation") {
            candle.animate(colorFlowAnimation);
        }
        else if (toAnimate == "FireAnimation") {
            candle.animate(fireAnimation);
        }
        else if (toAnimate == "LarsonAnimation") {
            candle.animate(larsonAnimation);
        }
        else if (toAnimate == "RainbowAnimation") {
            candle.animate(rainbowAnimation);
        }
        else if (toAnimate == "RgbFadeAnimation") {
            candle.animate(rgbFadeAnimation);
        }
        else if (toAnimate == "SingleFadeAnimation") {
            candle.animate(singleFadeAnimation);
        }
        else if (toAnimate == "StrobeAnimation") {
            candle.animate(strobeAnimation);
        }
        else if (toAnimate == "TwinkleAnimation") {
            candle.animate(twinkleAnimation);
        }
        else if (toAnimate == "TwinkleOffAnimation") {
            candle.animate(twinkleOffAnimation);
        }
    }

      
    public Command SetLEDOn() {
        return new FunctionalCommand(
            () -> System.out.println("LED on"), 
            () -> setAllLEDColor(LEDConstants.redRGB),
            (interrupted) -> setLEDOff(),
            () -> false, this);
    }

    public Command LedAnimate(String toAnimate) {
        return new FunctionalCommand(
            () -> System.out.println("Animate Start"), 
            () -> animate(toAnimate),
            (interrupted) -> setLEDOff(),
            () -> false, this);
    }
}