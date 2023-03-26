public class PID {
    private double P;
    private double I;
    private double D;
    private double integral;
    private double lastError;
    private boolean firstRun;

    public PID(double p, double i, double d) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.integral = 0;
        this.firstRun = true;
    }

    public double update(double error, double dt) {
        if (firstRun) {
            lastError = error;
            firstRun = false;
        }

        integral += I * error * dt;
        double diff = (error - lastError) / dt;
        double control_out = P * error + D * diff + integral * I;
        lastError = error;
        return  control_out;
    }


}
