
/**
 * This class represents the basic flight controller of the Bereshit space craft.
 *
 * @author ben-moshe
 */
public class Bereshit_101 {
    public static final double WEIGHT_EMP = 165; // kg
    public static final double WEIGHT_FULE = 420; // kg
    public static final double WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FULE; // kg
    // https://davidson.weizmann.ac.il/online/askexpert/%D7%90%D7%99%D7%9A-%D7%9E%D7%98%D7%99%D7%A1%D7%99%D7%9D-%D7%97%D7%9C%D7%9C%D7%99%D7%AA-%D7%9C%D7%99%D7%A8%D7%97
    public static final double MAIN_ENG_F = 430; // N
    public static final double SECOND_ENG_F = 25; // N
    public static final double MAIN_BURN = 0.15; //liter per sec, 12 liter per m'
    public static final double SECOND_BURN = 0.009; //liter per sec 0.6 liter per m'
    public static final double ALL_BURN = MAIN_BURN + 8 * SECOND_BURN;


    private static double getDhs(double alt) {
        double minAlt = 2000;
        double maxAlt = 30000;
        if (alt < minAlt)
            return 0;
        if (alt > maxAlt)
            return Moon.EQ_SPEED;

        double norm = (alt - minAlt) / (maxAlt - minAlt);
        return norm * Moon.EQ_SPEED;
    }

    private static double getDvs(double alt) {
        if (alt > 8000)
            return 30;
        if (alt > 500)
            return 24;
        if (alt > 300)
            return 12;
        if (alt > 100)
            return 6;

        if (alt > 50)
            return 3;

        if (alt > 25)
            return 2;

        return 1;
    }

    private static double getAng(double alt) {
        if (alt > 1500)
            return 60;
        if (alt > 1200)
            return 50;
        if (alt > 1000)
            return 30;
        return 0;
    }

    // 14095, 955.5, 24.8, 2.0
    public static void main(String[] args) {
        System.out.println("Simulating Bereshit's Landing:");
        // starting point:
        double vs = 24.8;
        double hs = 932;
        double dist = 181 * 1000;
        double ang = 58.3; // zero is vertical (as in landing)
        double alt = 13748; // 2:25:40 (as in the simulation) // https://www.youtube.com/watch?v=JJ0VfRL9AMs
        double time = 0;
        double dt = 1; // sec
        double acc = 0; // Acceleration rate (m/s^2)
        double fuel = 121; //
        double weight = WEIGHT_EMP + fuel;
        System.out.println("vs , desired_vs , hs , desired_hs , alt , ang , acc , fuel");
        double NN = 0.7; // rate[0,1]
        SpaceCraft craft = new SpaceCraft(vs, hs, ang, fuel, NN, dist, alt, time, dt, acc, weight);

        PID pid = new PID(0.014, 0.000000003, 0.2);
        PID pid_ang = new PID(0.314, 0.00003, 0.13);


        // ***** main simulation loop ******
        while (craft.getAlt() > 0) {
            double desired_vs = getDvs(craft.getAlt());
            double desired_hs = getDhs(craft.getAlt());
            double dsAngle = getAng(craft.getAlt());

            if (time % 10 == 0 || craft.getAlt() < 100) {
                System.out.println(craft.getAlt() + ", " + craft.getVs() + ", " + desired_vs + ", " + craft.getHs() + ", " + desired_hs + ", ang: " + craft.getAng() + " ,dang: " + dsAngle + ", " + craft.getAcc() + ", " + craft.getNN() + ", " + craft.getFuel());
            }

            double thrustIncr = pid.update(craft.getVs() - desired_vs + craft.getHs() - desired_hs, craft.getDt());
            double angIncr = pid_ang.update(dsAngle - craft.getAng(), craft.getDt());

            craft.increaseAngle(angIncr);
            craft.increaseThrust(thrustIncr);
            System.out.println("----------->" + angIncr + "  " + thrustIncr );

            craft.computeNextStep();
        }
        System.out.println(craft.getAlt() + ", " + craft.getVs() + ", " + craft.getHs() + ", " + craft.getAng() + ", " + craft.getAcc() + ", " + craft.getNN() + ", " + craft.getFuel());

    }
}
