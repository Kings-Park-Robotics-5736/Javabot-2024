package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
public class WPILEDSubsystem extends SubsystemBase{
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer; 
    private int m_rainbowFirstPixelHue;


    public WPILEDSubsystem(){


        // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(1);
    

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(5);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();



    }


    private void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255,255);
          System.out.println("led"+i + "hue:"+hue);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 1;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
      }



public void setAllLEDColor(int[] rgb) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, rgb[0], rgb[1], rgb[2]);
                System.out.println("Setting"+i);
            }
            m_led.setData(m_ledBuffer);
        }
        
public void setAllLEDColorHSV() {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setHSV(i, 100, 100, 100);
                System.out.println("Setting"+i);
            }
            m_led.setData(m_ledBuffer);
        }
public void setLEDOff(){

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 0, 0, 0);

     }
     
     m_led.setData(m_ledBuffer);
}



    public Command SetLEDOn() {
        return new FunctionalCommand(
            () -> System.out.println("LED on"), 
            () -> {setAllLEDColor(LEDConstants.redRGB);},
            (interrupted) -> setLEDOff(),
            () -> false, this); 
    }
        public Command SetLEDBOW() {
        return new FunctionalCommand(
            () -> System.out.println("LED on"), 
            () -> {rainbow();},
            (interrupted) -> setLEDOff(),
            () -> false, this); 
    }
}