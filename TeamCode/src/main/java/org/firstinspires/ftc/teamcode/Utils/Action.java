package org.firstinspires.ftc.teamcode.Utils;

public class Action implements Runnable {
    private final boolean isThread;
    public Runnable action;
    private boolean running = false;
    private boolean started = false;

    public Action(Runnable action, boolean isThread) {
        this.action = action;
        this.isThread = isThread;
    }

    public void run() {
        running = true;
        started = true;
        action.run();
        running = false;
    }

    public boolean isRunning() {
        return running;
    }

    public boolean hasStarted() {
        return started;
    }

    public boolean isThread() {
        return this.isThread;
    }

}