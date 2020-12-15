/*
Copyright (c) 2011 Tsz-Chiu Au, Peter Stone
University of Texas at Austin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the University of Texas at Austin nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package aim4.util;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

import aim4.config.Condor;
import aim4.config.Constants;
import aim4.driver.coordinator.V2ICoordinator;
import aim4.im.IntersectionManager;
import aim4.map.lane.Lane;
import aim4.vehicle.VehicleSimView;
import java.awt.geom.Point2D;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.NavigableMap;

/**
 * This class provides helper methods that are used throughout the code.
 */
public class Util {

    /////////////////////////////////
    // PUBLIC METHODS
    /////////////////////////////////
    /**
     * The random seed for all random number generators in the simulation
     */
    public static long randSeed = (new Random()).nextLong();
    // public static final long randSeed = -6397397808339168785L;

    /**
     * The global random number generator
     */
    public static final Random RANDOM_NUM_GEN = new Random();

    public static void resetRand(int seed) {
        randSeed = seed;
        RANDOM_NUM_GEN.setSeed(randSeed);
    }

    static {
        if (Condor.IS_CONDOR_EXIST) {
            // To make sure different processes on Condor receives different random
            // seeds, different processes sleep for a different time.
            try {
                Thread.sleep(Condor.CONDOR_ID * 10);
            } catch (InterruptedException e) {
                // ignore the interruption by another thread
            }
            randSeed = Util.RANDOM_NUM_GEN.nextLong() + Condor.CONDOR_ID;
            Util.RANDOM_NUM_GEN.setSeed(randSeed);
        }

//    if (Debug.IS_PRINT_RANDOM_SEED) {
//      System.err.println("randSeed = " + Util.randSeed + "L");
//    }
    }

    /////////////////////////////////
    // PUBLIC METHODS
    /////////////////////////////////
    //see http://floating-point-gui.de/errors/comparison
    /**
     * Check whether two double values are "nearly" equal.
     *
     * @param a the first double value
     * @param b the second double value
     * @param epsilon the precision
     * @return whether the two values are nearly equal.
     *
     */
    public static boolean nearlyEqual(double a, double b, double epsilon) {
        final double absA = Math.abs(a);
        final double absB = Math.abs(b);
        final double diff = Math.abs(a - b);

        if (a * b == 0) { // a or b or both are zero
            // relative error is not meaningful here
            return diff < (epsilon * epsilon);
        } else { // use relative error
            return diff / (absA + absB) < epsilon;
        }
    }

    /**
     * Whether a floating-point numbers (doubles) is zero.
     *
     * @param a a double value
     * @return whether the floating-point number is zero.
     */
    public static boolean isDoubleZero(double a) {
        return Math.abs(a) <= Constants.DOUBLE_EQUAL_PRECISION;
    }

    /**
     * Whether a floating-point numbers (doubles) is not zero.
     *
     * @param a a double value
     * @return whether the floating-point number is not zero.
     */
    public static boolean isDoubleNotZero(double a) {
        return Math.abs(a) > Constants.DOUBLE_EQUAL_PRECISION;
    }

    /**
     * Whether two floating-point numbers (doubles) are equal.
     *
     * @param a a double value
     * @param b a double value
     * @return whether the two floating-point numbers are equal.
     */
    public static boolean isDoubleEqual(double a, double b) {
        return Math.abs(a - b) <= Constants.DOUBLE_EQUAL_PRECISION;
    }

    /**
     * Whether two floating-point numbers (doubles) are not equal.
     *
     * @param a a double value
     * @param b a double value
     * @return whether the two floating-point numbers are not equal.
     */
    public static boolean isDoubleNotEqual(double a, double b) {
        return Math.abs(a - b) > Constants.DOUBLE_EQUAL_PRECISION;
    }

    /**
     * Whether two floating-point numbers (doubles) are equal.
     *
     * @param a a double value
     * @param b a double value
     * @param precision the bound on of the difference between a and b that is
     * considered equal.
     * @return whether the two floating-point numbers are equal.
     */
    public static boolean isDoubleEqual(double a, double b, double precision) {
        return Math.abs(a - b) <= precision;
    }

    /**
     * Whether two floating-point numbers (doubles) are not equal.
     *
     * @param a a double value
     * @param b a double value
     * @param precision the bound on of the difference between a and b that is
     * considered equal.
     * @return whether the two floating-point numbers are not equal.
     */
    public static boolean isDoubleNotEqual(double a, double b, double precision) {
        return Math.abs(a - b) > precision;
    }

    /**
     * Whether two floating-point numbers (doubles) are greater than or equal to
     * another.
     *
     * @param a a double value
     * @param b a double value
     * @return whether a >= b.
     */
    public static boolean isDoubleEqualOrGreater(double a, double b) {
        return a > b || isDoubleEqual(a, b);
    }

    /**
     * Whether two floating-point numbers (doubles) are less than or equal to
     * another.
     *
     * @param a a double value
     * @param b a double value
     * @return whether a <= b.
     */
    public static boolean isDoubleEqualOrLess(double a, double b) {
        return a < b || isDoubleEqual(a, b);
    }

    /**
     * Constrain a double value between a lower and upper bound. If it is below
     * the minimum value, return the minimum value. If it is above the maximum
     * value, return the maximum value. Otherwise, return the original value.
     *
     * @param inputValue the value to constrain
     * @param minValue the lower bound
     * @param maxValue the upper bound
     * @return a value between the lower and upper bounds
     */
    public static double constrain(double inputValue,
            double minValue, double maxValue) {
        if (inputValue > maxValue) {
            return maxValue;
        }
        if (inputValue < minValue) {
            return minValue;
        }
        return inputValue;
    }

    /**
     * Recenter a double value between a lower and upper bound. This means that
     * if a number is between the two bounds, it is returned unchanged, but if
     * it falls out of bounds, it is adjusted by the size of the interval until
     * it fits. For example, it can be used for for recentering angles between 0
     * and 2&pi; or 0 and 180.
     *
     * @param inputValue the value to recenter
     * @param minValue the lower bound
     * @param maxValue the upper bound
     * @return a value between the lower and upper bounds
     */
    public static double recenter(double inputValue,
            double minValue, double maxValue) {
        while (inputValue < minValue) {
            inputValue += (maxValue - minValue);
        }
        while (inputValue > maxValue) {
            inputValue -= (maxValue - minValue);
        }
        return inputValue;
    }

    /**
     * The sum of a sequence of floating-point numbers
     *
     * @param as an iterable of double values
     * @return the sum of the double values
     */
    public static double sum(Iterable<Double> as) {
        double sum = 0;
        for (double a : as) {
            sum += a;
        }
        return sum;
    }

    /**
     * The sum of an array of floating-point numbers
     *
     * @param as an array of double values
     * @return the sum of the double values
     */
    public static double sum(double[] as) {
        double sum = 0;
        for (double a : as) {
            sum += a;
        }
        return sum;
    }

    /**
     * Choose a number according to a finite probability distribution.
     *
     * @param distribution the probability distribution
     * @return an index of the distribution that is randomly chosen according to
     * the distribution
     */
    public static int randomIndex(double[] distribution) {
        double a = Util.RANDOM_NUM_GEN.nextDouble();
        for (int i = 0; i < distribution.length; i++) {
            a -= distribution[i];
            if (a < 0.0) {
                return i;
            }
        }
        throw new IllegalArgumentException("Invalid proportions.");
    }

    /**
     * Choose a number according to a finite probability distribution.
     *
     * @param distribution the probability distribution
     * @return an index of the distribution that is randomly chosen according to
     * the distribution
     */
    public static int randomIndex(List<Double> distribution) {
        double a = Util.RANDOM_NUM_GEN.nextDouble();
        for (int i = 0; i < distribution.size(); i++) {
            a -= distribution.get(i);
            if (a < 0.0) {
                return i;
            }
        }
        throw new IllegalArgumentException("Invalid proportions.");
    }

    /**
     * Concatenate a list of strings.
     *
     * @param strings a list of string
     * @param sep the separator
     * @return the concatenation of the list of string.
     */
    public static String concatenate(List<String> strings, String sep) {
        String str = "";
        for (String s : strings) {
            if (str.equals("")) {
                str = s;
            } else {
                str += sep + s;
            }
        }
        return str;
    }

    /**
     * Concatenate an array of strings.
     *
     * @param strings an array of string
     * @param sep the separator
     * @return the concatenation of the list of string.
     */
    public static String concatenate(String[] strings, String sep) {
        return concatenate(Arrays.asList(strings), sep);
    }

    /**
     * Read the content of a file into a list of strings.
     *
     * @param inFileName the name of the file
     * @return the list of strings
     * @throws IOException
     */
    public static List<String> readFileToStrArray(String inFileName) throws
            IOException {
        List<String> result = new LinkedList<String>();
        FileInputStream fstream = new FileInputStream(inFileName);
        // InputStream fstream = Util.class.getResourceAsStream(inFileName);
        if (fstream == null) {
            System.err.printf("Error: %s\n", inFileName);
        }
        DataInputStream in = new DataInputStream(fstream);
        BufferedReader br = new BufferedReader(new InputStreamReader(in));
        while (true) {
            String strLine = br.readLine();
            if (strLine == null) {
                break;
            }
            result.add(strLine);
        }
        in.close();
        return new ArrayList<String>(result);
    }

    /**
     * Gets a direction from a heading from a coordinate system mirrored across
     * the x-axis from the standard mathematical coordinate system, NOT a
     * navigational coordinate system. 0 radians is straight right (positive X),
     * pi/2 radians is down (negative y). This is to account for the way
     * road/lane lines are drawn in the simulator.
     *
     * @param head the heading in radians of the road.
     * @param map the map to check heading against direction
     * @return Enum indicating one of eight directions (North, northeast, etc.).
     */
    private static Constants.Direction getDirectionFromHeading(double head, NavigableMap<Double, Constants.Direction> map) {
        //ensuring angle is in the proper range
        double actualHead = Math.abs(head) % (Math.PI / 8);
        Double indexForHead = map.floorKey(head);
        //this happens because the RADIANS_TO_DIRECTION maps aren't cyclical. There's a "break" between when EAST starts (final entry) and when it ends (first entry).
        if (indexForHead == null) {
            indexForHead = map.lastKey();
        }
        return map.get(indexForHead);
    }

    /**
     * Gets a direction from a heading from a coordinate system mirrored across
     * the x-axis from the standard mathematical coordinate system, NOT a
     * navigational coordinate system. 0 radians is straight right (positive X),
     * pi/2 radians is down. This is to account for the way road/lane lines are
     * drawn in the simulator.
     *
     * @param head the heading in radians of the road.
     * @return Enum indicating one of eight directions (North, northeast, etc.).
     */
    public static Constants.Direction getDirectionFromHeadingCardinalOrIntercardinal(double head) {
        return getDirectionFromHeading(head, Constants.RADIANS_TO_DIRECTION_CARDINAL_OR_INTERCARDINAL_MAP);
    }

    /**
     * Gets a direction from a heading from a coordinate system mirrored across
     * the x-axis from the standard mathematical coordinate system, NOT a
     * navigational coordinate system. 0 radians is straight right (positive X),
     * pi/2 radians is down. This is to account for the way road/lane lines are
     * drawn in the simulator.
     *
     * @param head the heading in radians of the road.
     * @return Enum indicating one of four cardinal directions (North, south,
     * etc.).
     */
    public static Constants.Direction getDirectionFromHeadingCardinal(double head) {
        return getDirectionFromHeading(head, Constants.RADIANS_TO_DIRECTION_CARDINAL_DIRECTIONS_MAP);
    }

    public static Constants.TurnDirection selectRandomFromCompoundAction(Constants.TurnDirection act) {
        if (null != act) {
            switch (act) {
                case STRAIGHT_LEFT:
                    return Constants.THROUGH_LEFT_DECOMPOSED_ACTIONS[(Util.RANDOM_NUM_GEN.nextInt(Constants.THROUGH_LEFT_DECOMPOSED_ACTIONS.length))];
                case STRAIGHT_RIGHT:
                    return Constants.THROUGH_RIGHT_DECOMPOSED_ACTIONS[(Util.RANDOM_NUM_GEN.nextInt(Constants.THROUGH_RIGHT_DECOMPOSED_ACTIONS.length))];
                case STRAIGHT_ALL:
                    return Constants.THROUGH_ALL_DECOMPOSED_ACTIONS[(Util.RANDOM_NUM_GEN.nextInt(Constants.THROUGH_ALL_DECOMPOSED_ACTIONS.length))];
                default:
                    return act;
            }
        }
        return null;
    }

    public static Constants.TurnDirection[] getDecomposedActions(Constants.TurnDirection act) {
        if (null != act) {
            switch (act) {
                case STRAIGHT_LEFT:
                    return Constants.THROUGH_LEFT_DECOMPOSED_ACTIONS;
                case STRAIGHT_RIGHT:
                    return Constants.THROUGH_RIGHT_DECOMPOSED_ACTIONS;
                case STRAIGHT_ALL:
                    return Constants.THROUGH_ALL_DECOMPOSED_ACTIONS;
                default:
                    return new Constants.TurnDirection[]{act};
            }
        }
        return null;
    }

    public static Constants.TurnDirection[] getComposedActions(Constants.TurnDirection act) {
        if (null != act) {
            switch (act) {
                case LEFT:
                    return new Constants.TurnDirection[]{Constants.TurnDirection.STRAIGHT_LEFT, Constants.TurnDirection.STRAIGHT_ALL};
                case RIGHT:
                    return new Constants.TurnDirection[]{Constants.TurnDirection.STRAIGHT_RIGHT, Constants.TurnDirection.STRAIGHT_ALL};
                case STRAIGHT:
                    return new Constants.TurnDirection[]{Constants.TurnDirection.STRAIGHT_LEFT, Constants.TurnDirection.STRAIGHT_RIGHT, Constants.TurnDirection.STRAIGHT_ALL};
                default:
                    return new Constants.TurnDirection[]{act};
            }
        }
        return null;
    }

    public static <T extends Enum<T>> T getParseEnumFromStringCaseInsen(String stringEnum, Class<T> enumTypeToCheck) {
        String trimmedStrEnum = stringEnum.trim().toLowerCase();
        for (Enum<T> enumToParse : enumTypeToCheck.getEnumConstants()) {
            if (trimmedStrEnum.equalsIgnoreCase(enumToParse.name())) {
                return (T) enumToParse;
            }
        }

        String allowedEnums = "(";
        for (Enum<T> enumerator : enumTypeToCheck.getEnumConstants()) {
            allowedEnums += " " + enumerator.name();
        }
        allowedEnums += ")";

        throw new RuntimeException("Unable to parse provided enum: " + trimmedStrEnum + ". Valid vlaues are: " + allowedEnums);

    }

    public static Constants.TurnDirection getTurnDirectionFromArrivalAndDepartureLanesForCardinalDirections(Lane arrivalLane, Lane departureLane, IntersectionManager im) {
        if (im == null) {
            throw new IllegalArgumentException("Intersection manager in getTurnDirectionFromArrivalAndDepartureLanesForCardinalDirections cannot be null.");
        }

        double arrivalHeading = im.getIntersection().getEntryHeading(arrivalLane);
        double departureHeading = im.getIntersection().getEntryHeading(departureLane);

        if (arrivalHeading % (Math.PI / 2) != 0 || departureHeading % (Math.PI / 2) != 0) {
            throw new IllegalArgumentException("getTurnDirectionFromArrivalAndDepartureLanesForCardinalDirections only supports roads that go in cardinal directions at the moment (intersection entry headings are even multiples of Pi/2 radians). Headings were...arrival: " + arrivalHeading + ", departure heading: " + departureHeading);
        }

        Constants.Direction arrivalLaneDir = getDirectionFromHeadingCardinal(arrivalHeading);
        Constants.Direction departureLaneDir = getDirectionFromHeadingCardinal(departureHeading);

        boolean isLeftTurn = isCrossTurnForCardinalDirectionRoads(arrivalLaneDir, departureLaneDir);
        boolean isRightTurn = isWithTrafficTurnForCardinalDirectionRoads(arrivalLaneDir, departureLaneDir);
        Constants.TurnDirection td = null;
        if (isLeftTurn) {
            td = Constants.TurnDirection.LEFT;
        } else if (isRightTurn) {
            td = Constants.TurnDirection.RIGHT;
        } else {
            td = Constants.TurnDirection.STRAIGHT;
        }

        return td;
    }

    /**
     * Checks whether the outgoing direction is a cross turn from the incoming
     * direction provided both are one of the four cardinal directions
     *
     * @param incoming Direction of the incoming road as one of the cardinal
     * directions
     * @param outgoing Direction of the outgoing road as one of the cardinal
     * directions
     * @return true if the outgoing direction is a cross turn from the incoming
     * direction
     */
    public static boolean isCrossTurnForCardinalDirectionRoads(Constants.Direction incoming, Constants.Direction outgoing) {
        if (!Constants.CARDINAL_DIRECTIONS.contains(incoming) || !Constants.CARDINAL_DIRECTIONS.contains(outgoing)) {
            String in = (incoming == null ? "null" : incoming.name());
            String out = (outgoing == null ? "null" : outgoing.name());
            throw new RuntimeException("Directions passed to isCrossTurnForCardinalDirectionRoads must be one of the four cardinal directions. They were...incoming: " + in + " outgoing: " + out);
        }

        double incomingAngle = Constants.HEADING_ON_SCREEN_OF_DIRECTION_MAP_IN_RADIANS.get(incoming);
        double outgoingAngle = Constants.HEADING_ON_SCREEN_OF_DIRECTION_MAP_IN_RADIANS.get(outgoing);
        double newAngle;
        if (null == Constants.CROSS_TURN_DIRECTION) {
            throw new RuntimeException("Cross turn direction wasn't set to left or right. This is an unknown configuration. It was: null");
        } else {
            newAngle = getNewHeadingFromQuarterPiRadianTurns(incomingAngle, Constants.CROSS_TURN_DIRECTION);
        }

        return newAngle == outgoingAngle;
    }

    /**
     * Checks whether the outgoing direction is a with traffic turn from the
     * incoming direction provided both are one of the four cardinal directions
     *
     * @param incoming Direction of the incoming road as one of the cardinal
     * directions
     * @param outgoing Direction of the outgoing road as one of the cardinal
     * directions
     * @return true if the outgoing direction is a with traffic turn from the
     * incoming direction
     */
    public static boolean isWithTrafficTurnForCardinalDirectionRoads(Constants.Direction incoming, Constants.Direction outgoing) {
        if (!Constants.CARDINAL_DIRECTIONS.contains(incoming) || !Constants.CARDINAL_DIRECTIONS.contains(outgoing)) {
            String in = (incoming == null ? "null" : incoming.name());
            String out = (outgoing == null ? "null" : outgoing.name());
            throw new RuntimeException("Directions passed to isWithTrafficTurnForCardinalDirectionRoads must be one of the four cardinal directions. They were...incoming: " + in + " outgoing: " + out);
        }

        double incomingAngle = Constants.HEADING_ON_SCREEN_OF_DIRECTION_MAP_IN_RADIANS.get(incoming);
        double outgoingAngle = Constants.HEADING_ON_SCREEN_OF_DIRECTION_MAP_IN_RADIANS.get(outgoing);
        double newAngle;
        if (null == Constants.WITH_TRAFFIC_TURN_DIRECTION) {
            throw new RuntimeException("With traffic turn direction wasn't set to left or right. This is an unknown configuration. It was: null");
        } else {
            newAngle = getNewHeadingFromQuarterPiRadianTurns(incomingAngle, Constants.WITH_TRAFFIC_TURN_DIRECTION);
        }

        return newAngle == outgoingAngle;
    }

    public static double getNewHeadingFromTurnWithQuarterPiRadianTurns(double originalHeading, Constants.TurnDirection td) {
        return getNewHeadingFromQuarterPiRadianTurns(originalHeading, td);
    }

    /**
     * gets angle to for a turning action in quarter pi radian granularity
     * (useful in determining the correct angle for a left turn or right turn on
     * cardinal direction roads in intersections)
     *
     * @param originalHeading
     * @param td
     * @return
     */
    private static double getNewHeadingFromQuarterPiRadianTurns(double originalHeading, Constants.TurnDirection td) {
        double newAngle = originalHeading;
        double turnDirectionAdjustment;
        if (null == Constants.CROSS_TURN_DIRECTION) {
            throw new RuntimeException("Cross turn direction wasn't set to left or right. This is an unknown configuration. It was: null");
        } else if (null == td) {
            throw new RuntimeException("The provided turn direction wasn't set to left, right, or straight. It was: null");
        } else {
            switch (td) {
                case LEFT:
                    turnDirectionAdjustment = -Math.PI / 2; //left turn on the flipped across the x axis screen coordinate system (positive y is down)
                    break;
                case RIGHT:
                    turnDirectionAdjustment = Math.PI / 2; //right turn on the flipped across the x axis screen coordinate system (positive y is down)
                    break;
                case STRAIGHT:
                    turnDirectionAdjustment = 0;
                    break;
                default:
                    throw new RuntimeException("The provided turn direction wasn't set to left, right, or straight. It was: " + Constants.CROSS_TURN_DIRECTION.name());
            }
            newAngle = (originalHeading + turnDirectionAdjustment) % (2 * Math.PI);
            if (newAngle < 0) {
                newAngle = newAngle + 2 * Math.PI;
            }
        }
        return newAngle;
    }

    /**
     * This function works because a vehicle approaching the intersection is
     * restricted to one lane and will obey that lane's speed limit until
     * arriving at another vehicle to queue or until entering the intersection
     *
     * @param im
     * @param vehicle
     * @return the max speed allowed on the lane the vehicle will enter on if
     * it has not entered the intersection, otherwise, -1.
     */
    public static double getMaxSpeedForVehicleIfNotBeyondIntersectionEntrance(IntersectionManager im, VehicleSimView vehicle) {
        if (vehicle.getDriver().getState() == V2ICoordinator.State.V2I_CLEARING || vehicle.getDriver().getState() == V2ICoordinator.State.V2I_TERMINAL_STATE) {
            return -1;
        }
        Point2D carFrontPoint = vehicle.gaugePointAtMiddleFront(0);
        Point2D interEnterPoint = im.getIntersection().getEntryPoint(vehicle.getDriver().getEntryLane() == null ? vehicle.getDriver().getCurrentLane() : vehicle.getDriver().getEntryLane());
        double angleToIntersection = Math.atan2(-carFrontPoint.getX() + interEnterPoint.getX(), carFrontPoint.getY() - interEnterPoint.getY()) - Math.PI / 2; //getting angle and adjusting

        angleToIntersection = (angleToIntersection + (angleToIntersection > 0 ? 0 : 2 * Math.PI)) % (2 * Math.PI); //correcting to be positive angle if needed and bounding to 2*Pi maximum
        double speed = vehicle.getDriver().getCurrentLane().getSpeedLimit();
        if (Util.isDoubleEqual(angleToIntersection, vehicle.gaugeHeading(), 0.1)) {
            return speed;
        } else {
            return -1;
        }
    }
    
    /**
     *
     * @param im
     * @param vehicle
     * @return the distance of the vehicle to the entrance point in the
     * intersection managed by the provided IM.
     */
    public static double getDistanceOfVehicleFromLaneEntrancePoint(IntersectionManager im, VehicleSimView vehicle) {
        Point2D carFrontPoint = vehicle.gaugePointAtMiddleFront(0);
        Point2D interEnterPoint = im.getIntersection().getEntryPoint(vehicle.getDriver().getCurrentLane());
        return carFrontPoint.distance(interEnterPoint);
    }

    /**
     * Function assumes a single intersection and that the approaching vehicle
     * is currently traveling at the maximum speed the lane in which it is in
     * allows.
     *
     * @param im
     * @param vehicle
     * @param timeToArrivalMax the time under which the vehicle must be able to
     * arrive in order to
     * @return true if a vehicle has not entered or passed the provided
     * intersection and it could possibly arrive at the intersection within
     * (inclusive) timeToArrivalMax, false otherwise.
     */
    public static boolean vehicleCouldPossiblyArriveWithinTime(IntersectionManager im, VehicleSimView vehicle, double timeToArrivalMax) {
        double speed = getMaxSpeedForVehicleIfNotBeyondIntersectionEntrance(im, vehicle);
        if (speed == -1) {
            //vehicle has seemingly passed the intersection
            return false;
        }
        //this distance call only works because of the check above, otherwise the lane it checks could be incorrect.
        return getDistanceOfVehicleFromLaneEntrancePoint(im, vehicle) / speed <= timeToArrivalMax;
    }

/////////////////////////////////
// CLASS CONSTRUCTORS
/////////////////////////////////
    /**
     * This class should never be instantiated.
     */
    private Util() {
    }
;

}
