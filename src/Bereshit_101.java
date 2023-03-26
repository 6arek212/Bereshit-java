
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
        if (alt > 2000)
            return 24;
        if (alt > 500)
            return 12;
        if (alt > 100)
            return 8;
        return 2;
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

        PID pid_h = new PID(0.0004, 0.00003, 0.02);
        PID pid_v = new PID(0.004, 0.0003, 0.012);

        double thrustIncrV = 0;
        double thrustIncrH = 0;

        // ***** main simulation loop ******
        while (craft.getAlt() > 0) {
            double desired_vs = getDvs(craft.getAlt());
            double desired_hs = getDhs(craft.getAlt());


            if (time % 10 == 0 || craft.getAlt() < 100) {
                System.out.println(craft.getAlt() + ", " + craft.getVs() + ", " + desired_vs + ", " + craft.getHs() + ", " + desired_hs + ", " + craft.getAng() + ", " + craft.getAcc() + ", " + craft.getNN() + ", " + craft.getFuel());
            }


            if (craft.getHs() > 0) {
                thrustIncrH = pid_h.update( craft.getHs() - desired_hs , craft.getDt());
                craft.increaseThrust(thrustIncrH);
            } else {
                thrustIncrV = pid_v.update(craft.getVs() - desired_vs, craft.getDt());
                craft.increaseAngle(-3);
                craft.increaseThrust(thrustIncrV);
            }
            System.out.println("----------->" + thrustIncrH + "  " + thrustIncrV);


//            if (thrustIncrH < 0 && thrustIncrV > 0) {
//                craft.increaseAngle(-0.3 * craft.getDt());
//            } else if (thrustIncrH > 0 && thrustIncrV < 0) {
//                craft.increaseAngle(0.3 * craft.getDt());
//            }

            // over 2 km above the ground
//            if (craft.getAlt() > 2000) {    // maintain a vertical speed of [20-25] m/s
//                if (craft.getVs() > 25) {
//                    craft.increaseThrust(0.003 * dt);
//                } // more power for braking
//                if (craft.getVs() < 20) {
//                    craft.increaseThrust(-0.003 * dt);
//                } // less power for braking
//            }
//            // lower than 2 km - horizontal speed should be close to zero
//            else {
//                if (craft.getAng() >= 0) {
//                    craft.increaseAngle(-3);
//                } // rotate to vertical position.
//
//                thrustIncrH = pid_h.update(craft.getHs() - desired_hs, craft.getDt());
//                craft.increaseThrust(thrustIncrH);
//
//                if (craft.getAlt() < 125) { // very close to the ground!
//                    craft.setNN(1); // maximum braking!
//                    if (craft.getVs() < 5) {
//                        craft.setNN(0.7);
//                    } // if it is slow enough - go easy on the brakes
//                }
//            }
//            if (craft.getAlt() < 5) { // no need to stop
//                craft.setNN(0.4);
//            }

            craft.computeNextStep();
        }
        System.out.println(craft.getAlt() + ", " + craft.getVs()  + ", " + craft.getHs()  + ", " + craft.getAng() + ", " + craft.getAcc() + ", " + craft.getNN() + ", " + craft.getFuel());

    }
}
