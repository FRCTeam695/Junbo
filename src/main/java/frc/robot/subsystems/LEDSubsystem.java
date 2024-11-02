package frc.robot.subsystems;
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
        RED,
        YELLOW,
        GREEN,
        BLUE,
    }
    
    public void setColor(String color) {
        alphaLED.setLength(alphaLEDBuffer.getLength());
        switch (color) {
            case "r":
                for (int i = 0; i < 5; i++) alphaLEDBuffer.setRGB(i, 255, 0, 0);
            case "g":
                for (int i = 0; i < 5; i++) alphaLEDBuffer.setRGB(i, 0, 255, 0);
            case "y":
                for (int i = 0; i < 5; i++) alphaLEDBuffer.setRGB(i, 255, 255, 0);
            case "b":
                for (int i = 0; i < 5; i++) alphaLEDBuffer.setRGB(i, 0, 0, 255);
        }
        alphaLED.setData(alphaLEDBuffer);
        alphaLED.start();
    }
}
