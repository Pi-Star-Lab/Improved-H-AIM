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
package aim4.config;

import java.text.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.Set;
import java.util.TreeMap;

/**
 * A class to hold constants for the simulator.
 */
public final class Constants {

    /**
     * A NumberFormat for zero places after the decimal. Used a lot, so might as
     * well just define it once.
     */
    public static final NumberFormat ZERO_DEC = new DecimalFormat("#");

    /**
     * A NumberFormat for one place after the decimal. Used a lot, so might as
     * well just define it once.
     */
    public static final NumberFormat ONE_DEC = new DecimalFormat("#.0");

    /**
     * A NumberFormat for two places after the decimal. Used a lot, so might as
     * well just define it once.
     */
    public static final NumberFormat TWO_DEC = new DecimalFormat("#.00");

    /**
     * A NumberFormat for ten places after the decimal. Used a lot, so might as
     * well just define it once.
     */
    public static final NumberFormat TEN_DEC = new DecimalFormat("#.0000000000");

    /**
     * The number of second per hour
     */
    public static final int numOfSecondPerHour = 3600;

    /**
     * A NumberFormat that ensures a width of at least 8 with leading zeroes.
     */
    public static final NumberFormat LEADING_ZEROES
            = new DecimalFormat("00000000");

    /**
     * The size, in bits, of an integer on our theoretical platform. {@value}
     * bits.
     */
    public static final int INTEGER_SIZE = 32; // 32 bits

    /**
     * The size, in bits, of a double-precision floating point number on our
     * theoretical platform. {@value} bits.
     */
    public static final int DOUBLE_SIZE = 64; // 64 bits

    /**
     * The size, in bits, of a boolean value on our theoretical platform.
     * {@value} bits.
     */
    public static final int BOOLEAN_SIZE = 1; // 1 bit

    /**
     * The size, in bits, of an enumerated data type on our theoretical
     * platform. {@value} bits.
     */
    public static final int ENUM_SIZE = 8; // 8 bits

    /**
     * The number of bits per byte ({@value}).
     */
    public static final int BITS_PER_BYTE = 8;

    /**
     * The number of bytes per kilobyte ({@value}).
     */
    public static final int BYTES_PER_KB = 1024;

    /**
     * The number of bits per kilobyte ({@value}).
     */
    public static final int BITS_PER_KB = BYTES_PER_KB * BITS_PER_BYTE;

    /**
     * The number of bytes per megabyte ({@value}).
     */
    public static final int BYTES_PER_MB = BYTES_PER_KB * 1024;

    /**
     * The precision with which two double values are considered equal. The
     * equality of two double values a and b should be tested by using
     * Math.abs(a-b) < Constants.DOUBLE_EQUAL_PRECISION. {@value}
     */
    public static final double DOUBLE_EQUAL_PRECISION = 0.0000000001;

    /**
     * The precision with which two double values are considered equal. The
     * equality of two double values a and b should be tested by using
     * Math.abs(a-b) < Constants.DOUBLE_EQUAL_PRECISION. {@value}
     */
    public static final double DOUBLE_EQUAL_WEAK_PRECISION = 0.000001;

    /**
     * The eight directions: north, east, south, west, northwest, northeast,
     * southeast, southwest. If you add or remove one, make sure to adjust the
     * utility functions below which encode values based on them.
     */
    public enum Direction {
        /**
         * North
         */
        NORTH,
        /**
         * East
         */
        EAST,
        /**
         * South
         */
        SOUTH,
        /**
         * West
         */
        WEST,
        NORTHWEST,
        NORTHEAST,
        SOUTHEAST,
        SOUTHWEST
    };

    public static final Map<Direction, Double> HEADING_ON_SCREEN_OF_DIRECTION_MAP_IN_RADIANS = getHeadingOfDirectionOnScreenMapInRadians();

    public static final Map<Direction, Double> getHeadingOfDirectionOnScreenMapInRadians() {
        EnumMap<Direction, Double> map = new EnumMap<Direction, Double>(Direction.class);
        map.put(Direction.EAST, 0.0);
        map.put(Direction.SOUTHEAST, Math.PI*1/4);
        map.put(Direction.SOUTH, Math.PI*1/2);
        map.put(Direction.SOUTHWEST, Math.PI*3/4);
        map.put(Direction.WEST, Math.PI);
        map.put(Direction.NORTHWEST, Math.PI*5/4);
        map.put(Direction.NORTH, Math.PI*3/2);
        map.put(Direction.NORTHEAST, Math.PI*7/4);
        return Collections.unmodifiableMap(map);
    }

    public static final Set<Direction> CARDINAL_DIRECTIONS = Collections.unmodifiableSet(EnumSet.of(Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST));

    /**
     * Maps radians to Direction for N, NW, W, SW, etc. Done with a coordinate
     * system mirrored across the y-axis from the standard mathematical
     * coordinates, NOT navigation coordinates (ie, North is 270 degrees or
     * 1.5PI radians). This is to account for the way road/lane lines are drawn
     * in the simulator. 8 directional calculations for radians starting at
     * SOUTHEAST and going counterclockwise around the compass. This is handled
     * as, with element x, direction is in range [x, element after x).
     */
    public static final NavigableMap<Double, Direction> RADIANS_TO_DIRECTION_CARDINAL_OR_INTERCARDINAL_MAP = getDirectionRadianMap(true);

    /**
     * Maps radians to Direction for N, W, S, and E. Done with a coordinate
     * system mirrored across the y-axis from the standard mathematical
     * coordinates, NOT navigation coordinates (ie, North is 270 degrees or
     * 1.5PI radians). This is to account for the way road/lane lines are drawn
     * in the simulator. 4 directional calculations for radians starting at
     * SOUTHEAST and going counterclockwise around the compass. This is handled
     * as, with element x, direction is in range [x, element after x).
     */
    public static final NavigableMap<Double, Direction> RADIANS_TO_DIRECTION_CARDINAL_DIRECTIONS_MAP = getDirectionRadianMap(false);

    /**
     * Performs the setup for RADIANS_TO_DIRECTION_CARDINAL_OR_INTERCARDINAL_MAP or
 RADIANS_TO_DIRECTION_CARDINAL_DIRECTIONS_MAP. Done with a coordinate
     * system mirrored across the y-axis from the standard mathematical
     * coordinates, NOT navigation coordinates (ie, North is 270 degrees or
     * 1.5PI radians). This is to account for the way road/lane lines are drawn
     * in the simulator. 8 directional calculations for radians starting at
     * SOUTHEAST and going counterclockwise around the compass. This is handled
     * as, with element x, direction is in range [x, element after x).
     *
     * @param allEightDirections true for the map to include N, S, E, W, NE, NW,
     * SE, SW; false for the map to only include the four cardinal directions
     */
    private static NavigableMap<Double, Direction> getDirectionRadianMap(boolean allEightDirections) {
        TreeMap<Double, Direction> imTheMap = new TreeMap<Double, Direction>();
        if (allEightDirections) {
            imTheMap.put(Math.PI / 8, Direction.SOUTHEAST);
        }
        imTheMap.put(3 * Math.PI / 8, Direction.SOUTH);
        if (allEightDirections) {
            imTheMap.put(5 * Math.PI / 8, Direction.SOUTHWEST);
        }
        imTheMap.put(7 * Math.PI / 8, Direction.WEST);
        if (allEightDirections) {
            imTheMap.put(9 * Math.PI / 8, Direction.NORTHWEST);
        }
        imTheMap.put(11 * Math.PI / 8, Direction.NORTH);
        if (allEightDirections) {
            imTheMap.put(13 * Math.PI / 8, Direction.NORTHEAST);
        }
        imTheMap.put(15 * Math.PI / 8, Direction.EAST);
        return Collections.unmodifiableNavigableMap(imTheMap);
    }

    /**
     * Direction turning across traffic, left is typical of the United States
     * and countries with similar driving rules. Note, some functions use this and assume it will be left due to short development times.
     */
    public static final TurnDirection CROSS_TURN_DIRECTION = Constants.TurnDirection.LEFT;

    /**
     * Direction turning with traffic, right is typical of the United States and
     * countries with similar driving rules. Note, some functions use this and assume it will be right due to short development times.
     */
    public static final TurnDirection WITH_TRAFFIC_TURN_DIRECTION = Constants.TurnDirection.RIGHT;

    /**
     * Direction turning with traffic, right is typical of the United States and
     * countries with similar driving rules
     */
    public static final EnumSet<TurnDirection> ACTABLE_TURN_DIRECTIONS = EnumSet.of(TurnDirection.RIGHT, TurnDirection.STRAIGHT, TurnDirection.LEFT);

    /**
     * A direction to turn at an intersection.
     */
    public enum TurnDirection {
        /**
         * Left turn.
         */
        LEFT,
        /**
         * Right turn.
         */
        RIGHT,
        /**
         * Go straight (i.e. no turn).
         */
        STRAIGHT,
        /**
         * 180 degree turn back to the same road (currently not allowed).
         */
        U_TURN,
        /**
         * Used purely for mapping input files to turns. Resolves as STRAIGHT or
         * LEFT.
         */
        STRAIGHT_LEFT,
        /**
         * Used purely for mapping input files to turns. Resolves as STRAIGHT or
         * RIGHT.
         */
        STRAIGHT_RIGHT,
        /**
         * Used purely for mapping input files to turns. Resolves as STRAIGHT,
         * RIGHT, or LEFT.
         */
        STRAIGHT_ALL,
    }

    public static TurnDirection[] THROUGH_LEFT_DECOMPOSED_ACTIONS = {Constants.TurnDirection.LEFT, Constants.TurnDirection.STRAIGHT};
    public static TurnDirection[] THROUGH_RIGHT_DECOMPOSED_ACTIONS = {Constants.TurnDirection.RIGHT, Constants.TurnDirection.STRAIGHT};
    public static TurnDirection[] THROUGH_ALL_DECOMPOSED_ACTIONS = {Constants.TurnDirection.LEFT, Constants.TurnDirection.RIGHT, Constants.TurnDirection.STRAIGHT};
    public static TurnDirection[] ALL_COMPOSED_ACTION_LABELS = {Constants.TurnDirection.STRAIGHT_LEFT, Constants.TurnDirection.STRAIGHT_RIGHT, Constants.TurnDirection.STRAIGHT_ALL};

    
    /**
     * The three possible states of a traffic light: green, yellow, and red.
     */
    public enum LightStatus {
        /**
         * A green light means go.
         */
        GREEN,
        /**
         * Yellow light means a red light is coming soon.
         */
        YELLOW,
        /**
         * Red means stop.
         */
        RED
    };

}
