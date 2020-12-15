package aim4.config;

import aim4.config.ringbarrier.RingAndBarrier;
import java.util.Map;

import aim4.im.v2i.V2IManager;
import aim4.im.v2i.RequestHandler.ApproxNPhasesTrafficSignalRequestHandler.SignalController;
import aim4.map.GridMap;
import aim4.map.destination.DestinationSelector;
import aim4.map.lane.Lane;
import aim4.vehicle.VehicleSimView;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

/**
 * Make it possible to get the handler of objects in other classes FIXME This is
 * for implementation convenience, not a good design
 *
 * @author menie482
 *
 */
public class Resources {

    /**
     * which contains policy
     *
     * TODO only dealing with one im
     */
    public static V2IManager im;

    /**
     * The list of vehicles This reference is got from
     * AutoDriverOnlySimulator.java
     */
    public static Map<Integer, VehicleSimView> vinToVehicles;

    /**
     * This helps to get which destination a lane would lead to.
     */
    public static DestinationSelector destinationSelector;

    /**
     * This is a GridMap handler
     */
    public static GridMap map;

    /**
     * For updating red signal
     */
    public static Map<Integer, SignalController> signalControllers;

    /**
     * Ring and barrier for signals
     */
    public static RingAndBarrier ringAndBarrier;

    /**
     * FIX ME this really needs to get ripped out. The whole simulator might
     * need to be redesigned to use quad trees for positions and do
     * vision/sensor based checks for vehicles and IMs, etc. Should probably
     * have an even system so that vehicles that are waiting for a signal change
     * can just subscribe instead of spamming reservation requests. This tracks the
     * vehicles by VIN on each lane
     */
    public static HashMap<Lane, Set<Integer>> laneToVin = new HashMap<Lane, Set<Integer>>();

    /**
     * FIX ME this really needs to get ripped out. The whole simulator might
     * need to be redesigned to use quad trees for positions and do
     * vision/sensor based checks for vehicles and IMs, etc. Should probably
     * have an even system so that vehicles that are waiting for a signal change
     * can just subscribe instead of spamming reservation requests. This tracks the
     * last known lane for each vehicle.
     */
    public static HashMap<Integer, Lane> vinToLane = new HashMap<Integer, Lane>();

    /**
     * FIX ME this really needs to get ripped out. The whole simulator might
     * need to be redesigned to use quad trees for positions and do
     * vision/sensor based checks for vehicles and IMs, etc. Should probably
     * have an even system so that vehicles that are waiting for a signal change
     * can just subscribe instead of spamming reservation requests. This tracks the
     * vehicles by VIN that are traversing the intersection.
     */
    public static Set<Integer> traversingVehicles = new HashSet<Integer>();
}
