public class SpaceCraft {
    private static final double WEIGHT_EMP = 165; // kg

    private double vs;
    private double hs;
    private double ang;
    private double fuel;
    private double NN;
    private double dist;
    private double alt;
    private  double time;
    private  double dt;
    private  double acc;
    private double weight;

    public SpaceCraft(double vs, double hs, double ang, double fuel, double NN, double dist, double alt, double time, double dt, double acc, double weight) {
        this.weight = weight;
        this.acc = acc;
        this.dt = dt;
        this.time = time;
        this.alt = alt;
        this.dist = dist;
        this.vs = vs;
        this.hs = hs;
        this.ang = ang;
        this.fuel = fuel;
        this.NN = NN;
    }

    // increase thrust with [0,1] constrain
    public void increaseThrust(double inc) {
        double val = this.NN + inc;
        if (val >= 0 && val <= 1) {
            this.NN = val;
        }
        if (val > 1){
            this.NN = 1;
        }

        if (val < 0){
            this.NN = 0;
        }
    }

    // increase angle with [0,90] constrain
    public void increaseAngle(double angInc) {
        double val = this.ang + angInc;
        if (val >= 0 && val <= 90) {
            this.ang = val;
        }
        if (val > 90) {
            this.ang = 90;
        }

        if (val < 0) {
            this.ang = 0;
        }
    }


    public double accMax(double weight) {
        return acc(weight, true, 8);
    }


    public double acc(double weight, boolean main, int seconds) {
        double t = 0;
        if (main) {
            t += Bereshit_101.MAIN_ENG_F;
        }
        t += seconds * Bereshit_101.SECOND_ENG_F;
        double ans = t / weight;
        return ans;
    }


    // physics computations
    public void computeNextStep() {
        // main computations
        double ang_rad = Math.toRadians(ang);
        double h_acc = Math.sin(ang_rad) * acc;
        double v_acc = Math.cos(ang_rad) * acc;
        double vacc = Moon.getAcc(hs);
        time += dt;
        double dw = dt * Bereshit_101.ALL_BURN * NN;
        if (fuel > 0) {
            fuel -= dw;
            weight = WEIGHT_EMP + fuel;
            acc = NN * accMax(weight);
        } else { // ran out of fuel
            acc = 0;
        }

        v_acc -= vacc;
        if (hs > 0) {
            hs -= h_acc * dt;
        }
        dist -= hs * dt;
        vs -= v_acc * dt;
        alt -= dt * vs;

        if (hs < 2.5)
            hs = 0;
    }

    public double getVs() {
        return vs;
    }

    public void setVs(double vs) {
        this.vs = vs;
    }

    public double getHs() {
        return hs;
    }

    public void setHs(double hs) {
        this.hs = hs;
    }

    public double getAng() {
        return ang;
    }

    public void setAng(double ang) {
        this.ang = ang;
    }

    public double getFuel() {
        return fuel;
    }

    public void setFuel(double fuel) {
        this.fuel = fuel;
    }

    public double getNN() {
        return NN;
    }

    public void setNN(double NN) {
        this.NN = NN;
    }

    public double getDist() {
        return dist;
    }

    public double getAlt() {
        return alt;
    }

    public double getTime() {
        return time;
    }

    public double getDt() {
        return dt;
    }

    public double getAcc() {
        return acc;
    }

    public double getWeight() {
        return weight;
    }
}
