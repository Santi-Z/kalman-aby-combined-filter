import java.util.Random;
import java.lang.Math;

public class CombinedFilter {
    public static double accelTrueInput, accelEstimateInput, measurementMaxRandDiffInput, measurementVarianceInput, placingMaxRandDiffInput, placingVarianceInput;
    public static double accelTrue, velocityTrue, positionTrue;
    public static double accelEstimate, velocityEstimate, positionEstimate;
    public static double positionMeasurement, positionPrediction, estimateUncertainty, kalmanGain, betaGain, gammaGain, timeStep;
    public static double measurementVariance, measurementMaxRandDiff, placingVariance, placingMaxRandDiff, estimateUncertaintyRange, processNoise;
    public static double residual, residualSum, residualSums, averageResidualSum, measurementResidual, measurementResidualSum, measurementResidualSums, averageMeasurementResidualSum;
    public static int iterations, runs;

    public static double fixDouble(double input) {return Math.round(input * 1000000000000000d) / 1000000000000000d;}

    public static double nextPosition(double currentPosition, double currentVelocity, double currentAcceleration) {
        return (timeStep * currentVelocity) + (timeStep * timeStep / 2 * currentAcceleration) + currentPosition;
    }

    public static double nextVelocity(double currentVelocity, double currentAcceleration) {
        return (currentAcceleration * timeStep) + currentVelocity;
    }

    public static double kalmanGain(double estimateUncertainty, double measurementUncertainty) {
        return estimateUncertainty / (estimateUncertainty + measurementUncertainty);
    }

    public static double getMeasurement(double position, double oneSidedRange) {
        Random randomGen = new Random();
        double random = randomGen.nextDouble();
        int negatizer = 1;

        if (randomGen.nextBoolean()) negatizer = -1;
        return (random * oneSidedRange * negatizer) + position;
    }

    public static double estimateUncertainty(double kalmanGain, double lastEstimateUncertainty) {
        return (1 - kalmanGain) * lastEstimateUncertainty;
    }

    public static double uncertaintyExtrapolation(double processNoise, double lastEstimateUncertainty) {
        return lastEstimateUncertainty + processNoise;
    }

    public static double positionEstimate(double lastPositionEstimate) {
        return lastPositionEstimate + (kalmanGain * (positionMeasurement - lastPositionEstimate));
    }

    public static double velocityEstimate(double lastPositionEstimate, double lastVelocityEstimate) {
        return lastVelocityEstimate + (betaGain * ((positionMeasurement - lastPositionEstimate) / timeStep));
    }

    public static double accelerationEstimate(double lastAccelerationEstimate, double lastPositionEstimate) {
        return lastAccelerationEstimate + (gammaGain * ((positionMeasurement - lastPositionEstimate) / (0.5 * timeStep * timeStep)));
    }

    public static double predictPosition(double estimatedAccel, double estimatedVelocity, double estimatedPosition) {
        return nextPosition(estimatedPosition, estimatedVelocity, estimatedAccel);
    }

    public static void resetVariables() {
        positionMeasurement = 0;
        residual = 0;
        residualSum = 0;
        measurementResidual = 0;
        measurementResidualSum = 0;
        velocityTrue = 0;
        positionTrue = 0;
        velocityEstimate = 0;
        positionEstimate = 0;
        processNoise = 0.00001000;
        betaGain = 0.5;
        gammaGain = 0.5;
        accelTrue = accelTrueInput;
        accelEstimate = accelEstimateInput;
        measurementMaxRandDiff = measurementMaxRandDiffInput;
        measurementVariance = measurementVarianceInput;
        placingMaxRandDiff = placingMaxRandDiffInput;
        placingVariance = placingVarianceInput;
        estimateUncertainty = placingVariance;

        positionPrediction = predictPosition(accelEstimate, velocityEstimate, positionEstimate);
        estimateUncertainty = uncertaintyExtrapolation(processNoise, estimateUncertainty);
    }

    public static void loop(int iterations) {
        resetVariables();
        for (int i = 0; i < iterations; i++) {
            positionTrue = nextPosition(positionTrue, velocityTrue, accelTrue);
            velocityTrue = nextVelocity(velocityTrue, accelTrue);
            positionMeasurement = getMeasurement(positionTrue, measurementMaxRandDiff);
            measurementResidual = Math.abs(positionMeasurement - positionTrue);
            measurementResidualSum += measurementResidual;

            kalmanGain = kalmanGain(estimateUncertainty, measurementVariance);
            estimateUncertainty = estimateUncertainty(kalmanGain, estimateUncertainty);

            positionEstimate = positionEstimate(positionPrediction);
            residual = Math.abs(positionEstimate - positionTrue);
            residualSum += residual;
            velocityEstimate = velocityEstimate(positionEstimate, velocityEstimate);
            accelEstimate = accelerationEstimate(accelEstimate, positionEstimate);

            estimateUncertainty = uncertaintyExtrapolation(processNoise, estimateUncertainty);
            positionPrediction = predictPosition(accelEstimate, velocityEstimate, positionEstimate);

            estimateUncertaintyRange = Math.sqrt(estimateUncertainty) * 3;
            positionTrue = fixDouble(positionTrue);
        }
        residualSums += residualSum;
        measurementResidualSums += measurementResidualSum;
    }

    public static void loopRepeat(int runs) {
        for (int i = 0; i < runs; i++) {
            loop(iterations);
        }
        averageResidualSum = residualSums / runs;
        averageMeasurementResidualSum = measurementResidualSums / runs;
        System.out.println("Average Measurement Residual Sum = " + averageMeasurementResidualSum);
        System.out.println("Average Residual Sum = " + averageResidualSum);
        System.out.println("Filter Advantage = " + (100 - (averageResidualSum / averageMeasurementResidualSum * 100)));
    }

    public static void getInput() {
        accelTrueInput = 2.82;
        accelEstimateInput = 3.0;
        measurementMaxRandDiffInput = 0.01;
        measurementVarianceInput = (measurementMaxRandDiffInput / 3) * (measurementMaxRandDiffInput / 3);
        placingMaxRandDiffInput = 0.0254;
        placingVarianceInput = (placingMaxRandDiffInput / 3) * (placingMaxRandDiffInput / 3);
        timeStep = 0.02;
        iterations = 50;
        runs = 10000000; // 10 million
    }

    public static void main(String args[]) {
        getInput();
        loopRepeat(runs);
    }
}
