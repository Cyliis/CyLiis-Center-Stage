package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.teamcode.Math.LowPassFilter;

@Config
public class LeftPixelColorSensor {
    public ColorRangeSensor sensor;

    public static double whiteRedLow = 150, whiteGreenLow = 150, whiteBlueLow = 150;
    public static double whiteRedHigh = 2000, whiteGreenHigh = 2000, whiteBlueHigh = 2000;

    public static double greenRedLow, greenGreenLow, greenBlueLow;
    public static double greenRedHigh, greenGreenHigh, greenBlueHigh;

    public static double cheeseRedLow, cheeseGreenLow, cheeseBlueLow;
    public static double cheeseRedHigh, cheeseGreenHigh, cheeseBlueHigh;

    public static double purpleRedLow, purpleGreenLow, purpleBlueLow;
    public static double purpleRedHigh, purpleGreenHigh, purpleBlueHigh;

    private double r,g,b;

    public static double filterParameter = 0.9;

    private final LowPassFilter redFilter, greenFilter, blueFilter;

    public LeftPixelColorSensor(ColorRangeSensor sensor){
        this.sensor = sensor;
        redFilter = new LowPassFilter(filterParameter, sensor.red());
        greenFilter = new LowPassFilter(filterParameter, sensor.green());
        blueFilter = new LowPassFilter(filterParameter, sensor.blue());
    }

    boolean pixel = false;

    public void update(){
        r = redFilter.getValue(sensor.red());
        g = greenFilter.getValue(sensor.green());
        b = blueFilter.getValue(sensor.blue());

        pixel = funnyIsPixel();

        r = redFilter.getValue(sensor.red());
        g = greenFilter.getValue(sensor.green());
        b = blueFilter.getValue(sensor.blue());

        pixel = pixel || funnyIsPixel();

        r = redFilter.getValue(sensor.red());
        g = greenFilter.getValue(sensor.green());
        b = blueFilter.getValue(sensor.blue());

        pixel = pixel || funnyIsPixel();
    }

    public boolean isWhite(){
        return (r >= whiteRedLow && r <= whiteRedHigh) && (g >= whiteGreenLow && g <= whiteGreenHigh) && (b >= whiteBlueLow && b <= whiteBlueHigh);
    }

    public boolean isCheese(){
        return (r >= cheeseRedLow && r <= cheeseRedHigh) && (g >= cheeseGreenLow && g <= cheeseGreenHigh) && (b >= cheeseBlueLow && b <= cheeseBlueHigh);
    }

    public boolean isGreen(){
        return (r >= greenRedLow && r <= greenRedHigh) && (g >= greenGreenLow && g <= greenGreenHigh) && (b >= greenBlueLow && b <= greenBlueHigh);
    }

    public boolean isPurple(){
        return (r >= purpleRedLow && r <= purpleRedHigh) && (g >= purpleGreenLow && g <= purpleGreenHigh) && (b >= purpleBlueLow && b <= purpleBlueHigh);
    }

    public boolean isPixel(){
        return pixel;
    }

    private boolean funnyIsPixel(){
        return isWhite() || isPurple() || isCheese() || isGreen();
    }

    public boolean isPixelUpdate(){
        update();
        return pixel;
    }

    @Override
    public String toString(){
        return String.valueOf(r) + " " + String.valueOf(g) + " " + String.valueOf(b);
    }

}
