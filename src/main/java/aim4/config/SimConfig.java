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

import java.awt.geom.Point2D;

/**
 * The configuration of a simulation.
 */
public class SimConfig {

    public static enum SIGNAL_TYPE {
        DEFAULT,
        TRADITIONAL,
        RED_PHASE_ADAPTIVE,
        ONE_LANE_VERSION,
        REVISED_PHASE,
        HUMAN_ADAPTIVE,
        FULLY_ACTUATED
    }

    public static enum VOLUME_TYPE {
        FILE,
        RANDOM
    }

    public static enum VEHICLE_TYPE {
        AUTO,
        HUMAN,
        CRUISE,
        ADAPTIVE_CRUISE
    }

    public static double RED_PHASE_LENGTH = 0;

    /**
     * The specific type of fcfs_signal, if it's applied
     */
    public static SIGNAL_TYPE signalType;

    /**
     * How the traffic volume information is generated. Generally, it should be
     * read from file. When doing experiments on best green signal length, this
     * might need randomly generated to find the best green signal length for
     * different volume.
     */
    public static VOLUME_TYPE volumeType = VOLUME_TYPE.FILE;

    /**
     * Whether dedicated lanes are enabled.
     */
    public static int DEDICATED_LANES = 0;

    /**
     * The time the simulation should run. If it is less than or equal to zero,
     * the simulation will run forever.
     */
    public static double TOTAL_SIMULATION_TIME = -1.0;

    /**
     * The number of cycles per second ({@value}) at which the simulator runs.
     */
    public static final double CYCLES_PER_SECOND = 50.0;

    /**
     * The length of a time step (simulation time) in the simulator ({@value}
     * seconds).
     */
    public static final double TIME_STEP = 1 / CYCLES_PER_SECOND;

    /**
     * The length of a time step (simulation time) in the reservation grid
     * ({@value} seconds).
     */
    public static final double GRID_TIME_STEP = TIME_STEP;

    /**
     * How often the simulator should consider spawning vehicles.
     */
    public static final double SPAWN_TIME_STEP = TIME_STEP / 10;

    /**
     * The portion of human drivers This data shoule be passed through command
     * line for experiment.
     */
    public static double HUMAN_PERCENTAGE = 0;

    /**
     * These percentage of drivers are told by the IM whether they should slow
     * down or speed up. Sure, this info is inquired only when human_percentage
     * > 0.
     */
    public static double CONSTANT_HUMAN_PERCENTAGE = 0;

    /**
     * The percentage of drivers who can strictly follow the vehicles in front
     * of it.
     */
    public static double ADAPTIVE_HUMAN_PERCENTAGE = 0;

    /**
     * Allowing the assumption that the IM can also have the information of the
     * positions of the human-driven vehicles.
     */
    public static boolean FULLY_OBSERVING = true;

    /**
     * times for human of time buffer NOT IN USE.
     */
    public static final double HUMAN_TARDINESS = 2;

    /**
     * This deals with a specific situation in FCFS-SIGNAL. The autonomous
     * vehicles have to check what's going on at time of
     * currentTime+GREEN_LANE_PREDICTION_BUFFER, too see whether it would
     * collides into a human vehicle, coming from a green lanes, which just
     * turned from a red lane.
     */
    public static final int GREEN_LANE_PREDICTION_BUFFER = 3;

    /**
     * when green lights are on in two opposite directions, whether we allow
     * vehicles turning left. otherwise, it's only permitted when light in one
     * road is on.
     */
    public static final boolean ALWAYS_ALLOW_TURNING_LEFT = false;
    /**
     * The longest time that the intersection should wait for human driver to go
     * through. This is the value that should be compared with the simulated
     * time for the driver of course, it cannot be infinite.
     */
    public static final double LONGEST_TIME_TO_WAIT_FOR_HUMAN = 0;

    /**
     * This is for for FCFS-SIGNAL The time that vehicles in the red lanes
     * before it arrives at the intersection
     */
    public static final double TIME_RED_LANE_CAN_PROPOSE = 0.7;

    /**
     * Whether or not the vehicle must stop before an intersection
     */
    public static boolean MUST_STOP_BEFORE_INTERSECTION = false;

    /**
     * The distance before the stopping distance before an intersection such
     * that a vehicle can consider moving again when
     * MUST_STOP_BEFORE_INTERSECTION is true.
     */
    public static final double ADDITIONAL_STOP_DIST_BEFORE_INTERSECTION = 0.01;

    /**
     * If an adaptive vehicle find a vehicle in front of it within such
     * distance, it can follow
     */
    public static final double FOLLOW_DISTANTCE = 15;

    /**
     * If an actuated signal will gapout early. Potentially due to the nature of the future error lookup window, this is ineffective and should remain as false.
     */
    public static boolean SIM_ALLOWS_EARLY_GAPOUT = false;
    
    /**
     * If actuation is allowed at all
     */
    public static boolean ALLOW_ACTUATION = true;
    
    /**
     * If actuation is allowed at all
     */
    public static boolean USE_ADAPTIVE_TIMING = true;
    
    /**
     * Flag for if explicit mappings should be used for which lanes should turn right on red (also called with traffic turns), or if the simulator should try to figure out what's allowed
     */
    public static boolean useExplicitMappingsForWithTrafficTurnOnRed = false;
    
    public static final boolean ALLOW_RIGHT_TURNS_ON_RED_FOR_ANY_VEHICLE_TYPE = true;
}
