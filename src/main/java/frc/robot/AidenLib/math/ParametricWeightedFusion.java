package frc.robot.AidenLib.math;

import java.util.ArrayList;

/** A class which uses multiple {@link WeightedFusion} objects to treat the values of a system parametrically. */
public class ParametricWeightedFusion {

    public class InvalidParamterSize extends Exception {
        public InvalidParamterSize(int required, int provided) {
            super(String.format("Requires parameters: %d; Provided paramters: %d", required, provided));
        }
    }

    ArrayList<WeightedFusion> filters = new ArrayList<WeightedFusion>();

    double[] val;

    /**
     * Constructs a {@link ParametricWeightedFusion} object.
     * @param numParamaters The number of parameters of the system. (e.g. A position system has 3 parameters: [x, y, z])
     */
    public ParametricWeightedFusion(int numParamaters) {
        for (int i = 0; i < numParamaters; i++) {
            filters.add(new WeightedFusion());
        }

        val = new double[filters.size()];
        for (int i = 0; i < val.length; i++) {
            val[i] = 0.0;
        }
    }

    /**
     * This method distributes the state estimates for each of the system parameters to their respective {@link WeightedFusion} objects.
     * @param estimates An array with a length equal to the number of parameters, made up of {@link Data} arrays.
     * @return The estimates for each parameter.
     * @throws InvalidParamterSize This is thrown when the length of the input array is not equal to the number of parameters.
     */
    public double[] calculate(Data[]... estimates) throws InvalidParamterSize {
        if (estimates.length != filters.size()) throw new InvalidParamterSize(filters.size(), estimates.length);

        for (int i = 0; i < estimates.length; i++) {
            val[i] = filters.get(i).calculate(estimates[i]);
        }

        return val;
    }
}
