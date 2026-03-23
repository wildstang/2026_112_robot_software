package org.wildstang.sample.subsystems.LED;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.Input;
import org.wildstang.framework.subsystems.Subsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.wildstang.sample.robot.WsSubsystems;
import org.wildstang.sample.subsystems.intake.Intake;
import org.wildstang.sample.subsystems.launcher.Launcher;
import org.wildstang.sample.subsystems.swerve.SwerveDrive;


public class LedController implements Subsystem {

    private Launcher launcher; 
    private Intake intake; 
    private SwerveDrive swerve;

    private static final int port = 0;
    private static final int length = 39;
    private static final double patternUpdateInterval = 0.1;
    private static enum LEDStates {BLUE_RAINBOW};

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private Timer patternUpdateClock = new Timer();
    private LEDStates ledState = LEDStates.BLUE_RAINBOW;

    private HueGradient blueRainbow = new HueGradient(85, 120, 255, 255);

    @Override
    public void init() {
        led = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);
        led.setLength(ledBuffer.getLength());
        setRGB(0, 0, 0);
        led.start();
        patternUpdateClock.start();
        
        intake = (Intake) Core.getSubsystemManager().getSubsystem(WsSubsystems.INTAKE);
        launcher = (Launcher) Core.getSubsystemManager().getSubsystem(WsSubsystems.LAUNCHER);
        swerve = (SwerveDrive) Core.getSubsystemManager().getSubsystem(WsSubsystems.SWERVE_DRIVE);
        
    }

    @Override
    public void inputUpdate(Input source) {
    }
    
    @Override
    public void initSubsystems() {
    }

    @Override
    public void update() {
        
        if(launcher.isRunning()) {
            ledBuffer.setRGB(length, 0, 225, 225);
        }
        else if(swerve.currentDriveState() == "auto") {
            ledBuffer.setRGB(length, 0, 0, 139);
        } else if(swerve.currentDriveState() == "teleop") {
            ledBuffer.setRGB(length, 0, 0, 225);
        } else if(swerve.currentDriveState() == "launch") {
            ledBuffer.setRGB(length, 173, 216, 230);
        } else if(swerve.currentDriveState() == "cross") {
            ledBuffer.setRGB(length, 225, 0, 0);
        } else if(swerve.currentDriveState() == "snake") {
            ledBuffer.setRGB(length, 0, 225, 0);
        } else if(swerve.currentDriveState() == "bump") {
            ledBuffer.setRGB(length, 0, 0, 225);
        } else if(swerve.currentDriveState() == "feed") {
            ledBuffer.setRGB(length, 135, 206, 235);
        }

        if (patternUpdateClock.hasElapsed(patternUpdateInterval)) {
            switch (ledState) {
                case BLUE_RAINBOW:
                    blueRainbow.update();
                    break;
            }
        }
        SmartDashboard.putString("LED state", ledState.toString());
    }

    public void setRGB(int red, int green, int blue) {
        for (int i = 0; i < length; i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
        led.setData(ledBuffer);
    }

    @Override
    public void selfTest() {
    }

    @Override
    public void resetState() {
    }

    @Override
    public String getName() {
        return ("LED Controller");
    }

    private class HueGradient {
        private int startHue;
        private int endHue;
        private int saturation;
        private int value;
        private double hueStep;
        private int inflectionPoint;

        private int offset = 0;

        public HueGradient(int startHue, int endHue, int saturation, int value) {
            this.startHue = startHue;
            this.endHue = endHue;
            this.saturation = saturation;
            this.value = value;
            this.hueStep = (endHue - startHue) / (length / 2.0);
            this.inflectionPoint = length / 2 + 1;
        }

        public void update() {
            for (int i = 0; i < inflectionPoint; i++) {
                ledBuffer.setHSV((i + offset) % length, (int) (startHue + (hueStep * i)), saturation, value);
            }
            for (int i = inflectionPoint; i < length; i++) {
                ledBuffer.setHSV((i + offset) % length, (int) (endHue - (hueStep * (i - inflectionPoint + 1))), saturation, value);
            }
            offset = (offset + 1) % length;
            led.setData(ledBuffer);
        }
    }
}
