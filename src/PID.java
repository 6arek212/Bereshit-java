public class PID {
    private double P;

    private double I;
    private double D;
    private double lastError;
    private boolean firstRun;

    private double integralError;

    public PID(double p, double i, double d) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.firstRun = true;
        this.integralError = 0;
    }

    public double update(double error, double dt) {
        if (this.firstRun) {
            this.firstRun = false;
            this.lastError = error;
        }
        double diff = (error - this.lastError) / dt;
        integralError += error * dt;
        double controlOut = P * error + I * integralError + D * diff;
        this.lastError = error;
        return controlOut;
    }


    public void reset() {
        this.firstRun = true;
        this.integralError = 0;
    }

}
