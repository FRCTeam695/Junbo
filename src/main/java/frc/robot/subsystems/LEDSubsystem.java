package frc.robot.subsystems;
import javax.swing.plaf.synth.Region;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
    AddressableLED alphaLED;
    AddressableLEDBuffer alphaLEDBuffer;

    public LEDSubsystem () {
        alphaLED = new AddressableLED(1);
        alphaLEDBuffer = new AddressableLEDBuffer(5);
    }

    public void motorLED(double angle) {
        int LEDpar = (int) (angle * 5.99); // -5 - 5
        alphaLED.setLength(alphaLEDBuffer.getLength());

        // Conditions for motor velocity
        if (LEDpar > 0) 
            for (int i = 0; i < LEDpar; i++) alphaLEDBuffer.setRGB(i, 255, 0, 0);

        else if (LEDpar < 0) 
            for (int i = 0; i < -LEDpar; i++) alphaLEDBuffer.setRGB(i, 0, 255, 0);

        else
            for (int i = 0; i < 5; i++) alphaLEDBuffer.setRGB(i, 0, 0, 0);

        // "Deploying" LED
        alphaLED.setData(alphaLEDBuffer);
        alphaLED.start();
    }
    public enum Colors {
        RED ("red", 255, 0, 0),
        YELLOW ("yellow", 255, 255, 0),
        GREEN ("green", 0, 255, 0),
        CYAN ("cyan", 0, 255, 255),
        BLUE ("blue", 0, 0, 255),
        PINK ("pink", 255, 0, 255);

        final String name;
        final int red;
        final int green;
        final int blue;

        Colors (String name, int red, int green, int blue) {
            this.name = name;
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }
    
    public void setColor(String color) {
        alphaLED.setLength(alphaLEDBuffer.getLength());
        for (Colors myColor : Colors.values()) {
            if (myColor.name.equals(color))
                for (int i = 0; i < 5; i++)
                    alphaLEDBuffer.setRGB(i, myColor.red, myColor.green, myColor.blue);
        }
        alphaLED.setData(alphaLEDBuffer);
        alphaLED.start();
    }
}
