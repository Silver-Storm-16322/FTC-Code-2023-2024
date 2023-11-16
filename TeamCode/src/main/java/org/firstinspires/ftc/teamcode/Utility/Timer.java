package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Timer {
    private final ElapsedTime timer = new ElapsedTime();
    private final TimeUnit timeUnit;
    private int time = 0;

    /**
     * Initializes the Timer with the specified duration and time unit.
     *
     * @param time The duration to wait for.
     * @param timeUnit The unit of time for the specified duration.
     */
    public Timer(int time, TimeUnit timeUnit) {
        this.time = time;
        this.timeUnit = timeUnit;
    }

    /**
     * Starts the timer.
     */
    public void start() {
        timer.startTime();
    }

    /**
     * Checks if the wait duration has elapsed.
     *
     * @return True if the specified time has passed since the command started.
     */
    public boolean done() {
        return (time <= timer.time(timeUnit));
    }
}
