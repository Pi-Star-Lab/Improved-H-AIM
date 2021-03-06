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
package aim4.sim;

import java.awt.Color;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Queue;
import java.util.Set;
import java.util.SortedMap;
import java.util.TreeMap;

import aim4.config.Debug;
import aim4.config.DebugPoint;
import aim4.config.RedPhaseData;
import aim4.config.Resources;
import aim4.config.SimConfig;
import aim4.config.SimConfig.SIGNAL_TYPE;
import aim4.config.SimConfig.VEHICLE_TYPE;
import aim4.config.ringbarrier.RingAndBarrier;
import aim4.driver.AutoDriver;
import aim4.driver.Driver;
import aim4.driver.DriverSimView;
import aim4.driver.ProxyDriver;
import aim4.driver.coordinator.V2ICoordinator;
import aim4.driver.coordinator.V2ICoordinator.State;
import aim4.im.IntersectionManager;
import aim4.im.v2i.V2IManager;
import aim4.im.v2i.RequestHandler.ApproxNPhasesTrafficSignalRequestHandler;
import aim4.im.v2i.RequestHandler.ApproxNPhasesTrafficSignalRequestHandler.CyclicSignalController;
import aim4.im.v2i.RequestHandler.ApproxNPhasesTrafficSignalRequestHandler.SignalController;
import aim4.im.v2i.policy.BasePolicy;
import aim4.map.DataCollectionLine;
import aim4.map.BasicMap;
import aim4.map.GridMapUtil;
import aim4.map.Road;
import aim4.map.SpawnPoint;
import aim4.map.SpawnPoint.SpawnSpec;
import aim4.map.SpawnPoint.SpawnSpecGenerator;
import aim4.map.lane.Lane;
import aim4.map.trafficbyturns.DestinationFileSpawnSpecGenerator;
import aim4.map.trafficbyturns.FileSpawnSpecGenerator;
import aim4.map.trafficbyturns.TurnMovements;
import aim4.msg.i2v.I2VMessage;
import aim4.msg.v2i.V2IMessage;
import aim4.sim.setup.AdaptiveTrafficSignalSuperviser;
import aim4.vehicle.AutoVehicleSimView;
import aim4.vehicle.BasicAutoVehicle;
import aim4.vehicle.HumanDrivenVehicleSimView;
import aim4.vehicle.ProxyVehicleSimView;
import aim4.vehicle.VehicleSpec;
import aim4.vehicle.VinRegistry;
import aim4.vehicle.VehicleSimView;
import java.util.Collections;
import expr.trb.DesignatedLanesExpr;
import expr.trb.TrafficSignalExpr;
import java.awt.geom.Area;

/**
 * The autonomous drivers only simulator.
 */
public class AutoDriverOnlySimulator implements Simulator {

    /////////////////////////////////
    // NESTED CLASSES
    /////////////////////////////////
    /**
     * The result of a simulation step.
     */
    public static class AutoDriverOnlySimStepResult implements SimStepResult {

        /**
         * The VIN of the completed vehicles in this time step
         */
        List<Integer> completedVINs;

        /**
         * Create a result of a simulation step
         *
         * @param completedVINs the VINs of completed vehicles.
         */
        public AutoDriverOnlySimStepResult(List<Integer> completedVINs) {
            this.completedVINs = completedVINs;
        }

        /**
         * Get the list of VINs of completed vehicles.
         *
         * @return the list of VINs of completed vehicles.
         */
        public List<Integer> getCompletedVINs() {
            return completedVINs;
        }
    }
    /**
     * The map
     */
    private BasicMap basicMap;
    /**
     * All active vehicles, in form of a map from VINs to vehicle objects.
     */
    public Map<Integer, VehicleSimView> vinToVehicles;
    /**
     * The current time
     */
    private double currentTime;
    /**
     * The number of completed vehicles
     */
    private int numOfCompletedVehicles;
    /**
     * The total number of bits transmitted by the completed vehicles
     */
    private int totalBitsTransmittedByCompletedVehicles;
    /**
     * The total number of bits received by the completed vehicles
     */
    private int totalBitsReceivedByCompletedVehicles;
    /**
     * The number of vehicles that are inhibited because of no enough space
     */
    private int inhibitedVehicles = 0;
    /**
     * The number of vehicles generated
     */
    public int generatedVehicles = 0;

    private List<Double> completionTimes;

    /**
     * Specifies flow and turn movement information
     */
    protected TurnMovements turnMovements;

    /////////////////////////////////
    // CLASS CONSTRUCTORS
    /////////////////////////////////
    /**
     * Create an instance of the simulator.
     *
     * @param basicMap the map of the simulation
     * @param turnMovements Specifies flow and turn movement information
     */
    public AutoDriverOnlySimulator(BasicMap basicMap, TurnMovements turnMovements) {
        this.basicMap = basicMap;
        this.vinToVehicles = new HashMap<Integer, VehicleSimView>();
        Resources.vinToVehicles = this.vinToVehicles;

        this.turnMovements = turnMovements;

        currentTime = 0.0;
        this.completionTimes = new ArrayList<Double>(5000);
        numOfCompletedVehicles = 0;
        totalBitsTransmittedByCompletedVehicles = 0;
        totalBitsReceivedByCompletedVehicles = 0;
    }

    /////////////////////////////////
    // PUBLIC METHODS
    /////////////////////////////////
    // the main loop
    /**
     * {@inheritDoc}
     */
    @Override
    public synchronized AutoDriverOnlySimStepResult step(double timeStep) {
        if (Debug.PRINT_SIMULATOR_STAGE) {
            System.err.printf("--------------------------------------\n");
            System.err.printf("------SIM:spawnVehicles---------------\n");
        }

        // update red signal for dynamic FCFS-SIGNAL 
        if (SimConfig.signalType == SimConfig.SIGNAL_TYPE.RED_PHASE_ADAPTIVE
                && ApproxNPhasesTrafficSignalRequestHandler.CyclicSignalController.needRecalculate(currentTime)) {
            updateTrafficSignal();
        } else if (SimConfig.signalType == SimConfig.SIGNAL_TYPE.FULLY_ACTUATED) {
            Resources.ringAndBarrier.refreshSignals(currentTime);
        }

        // spawning vehicles from spawning points according to traffic level
        spawnVehicles(timeStep);
        if (Debug.PRINT_SIMULATOR_STAGE) {
            System.err.printf("------SIM:provideSensorInput---------------\n");
        }
        
        // generate information like the linked-table of vehicles, intervals between vehicles, signal, etc 
        provideSensorInput();
        if (Debug.PRINT_SIMULATOR_STAGE) {
            System.err.printf("------SIM:letDriversAct---------------\n");
        }

        // allow driver proposals
        letDriversAct();
        if (Debug.PRINT_SIMULATOR_STAGE) {
            System.err.printf("------SIM:letIntersectionManagersAct--------------\n");
        }
        /*for (VehicleSimView v1 : vinToVehicles.values()) {
            for (VehicleSimView v2 : vinToVehicles.values()) {
                Area a1 = new Area(v1.getShape());
                Area a2 = new Area(v2.getShape());
                a1.intersect(a2);
                if (v1 != v2 && !a1.isEmpty()) {
                    System.out.println(v1.getFrontVehicle());
                    System.out.println(v2.getFrontVehicle());
                    throw new RuntimeException("ded " + currentTime + " " + v1.getVIN() + " " + v2.getVIN());
                }
            }
        }*/

        // intersection dealing with proposals
        letIntersectionManagersAct(timeStep);
        if (Debug.PRINT_SIMULATOR_STAGE) {
            System.err.printf("------SIM:communication---------------\n");
        }

        communication();
        if (Debug.PRINT_SIMULATOR_STAGE) {
            System.err.printf("------SIM:moveVehicles---------------\n");
        }

        // move vehicles graphically. DCL information calculated here.
        moveVehicles(timeStep);
        if (Debug.PRINT_SIMULATOR_STAGE) {
            System.err.printf("------SIM:cleanUpCompletedVehicles---------------\n");
        }

        // for human-adaptive traffic signals, run green phases periodically
        if (SimConfig.signalType == SIGNAL_TYPE.HUMAN_ADAPTIVE) {
            if (currentTime % AdaptiveTrafficSignalSuperviser.getPhaseLength() < timeStep) {
                AdaptiveTrafficSignalSuperviser.runGreenLight(currentTime);
            }
        }

        List<Integer> completedVINs = cleanUpCompletedVehicles();
        currentTime += timeStep;
        // debug
        checkClocks();

        if (turnMovements != null && turnMovements.getExpectedSpawnsUpToEndOfTimeSlot(currentTime) != (getGeneratedVehiclesNum() + getScheduledVehiclesRemaining())) {
            throw new RuntimeException("Simulation failed at time: " + currentTime + " Number of vehicles spawned and/or scheduled are not the expected numbers of vehicles.\n"
                    + "Expected by end of time slot: " + turnMovements.getExpectedSpawnsUpToEndOfTimeSlot(currentTime) + "\n"
                    + "Number currently spawned and/or scheduled: " + (getGeneratedVehiclesNum() + getScheduledVehiclesRemaining()));
        }

        return new AutoDriverOnlySimStepResult(completedVINs);
    }

    /////////////////////////////////
    // PUBLIC METHODS
    /////////////////////////////////
    // information retrieval
    /**
     * {@inheritDoc}
     */
    @Override
    public synchronized BasicMap getMap() {
        return basicMap;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public synchronized double getSimulationTime() {
        return currentTime;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public synchronized int getNumCompletedVehicles() {
        return numOfCompletedVehicles;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public synchronized double getAvgBitsTransmittedByCompletedVehicles() {
        if (numOfCompletedVehicles > 0) {
            return ((double) totalBitsTransmittedByCompletedVehicles)
                    / numOfCompletedVehicles;
        } else {
            return 0.0;
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public synchronized double getAvgBitsReceivedByCompletedVehicles() {
        if (numOfCompletedVehicles > 0) {
            return ((double) totalBitsReceivedByCompletedVehicles)
                    / numOfCompletedVehicles;
        } else {
            return 0.0;
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public synchronized Set<VehicleSimView> getActiveVehicles() {
        return new HashSet<VehicleSimView>(vinToVehicles.values());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public VehicleSimView getActiveVehicle(int vin) {
        return vinToVehicles.get(vin);
    }

    /////////////////////////////////
    // PUBLIC METHODS
    /////////////////////////////////
    /**
     * {@inheritDoc}
     */
    @Override
    public synchronized void addProxyVehicle(ProxyVehicleSimView vehicle) {
        Point2D pos = vehicle.getPosition();
        Lane minLane = null;
        double minDistance = -1.0;

        for (Road road : basicMap.getRoads()) {
            for (Lane lane : road.getLanes()) {
                double d = lane.nearestDistance(pos);
                if (minLane == null || d < minDistance) {
                    minLane = lane;
                    minDistance = d;
                }
            }
        }
        assert minLane != null;

        ProxyDriver driver = vehicle.getDriver();
        if (driver != null) {
            driver.setCurrentLane(minLane);
            driver.setSpawnPoint(null);
            driver.setDestination(null);
        }

        vinToVehicles.put(vehicle.getVIN(), vehicle);
    }

    /////////////////////////////////
    // PRIVATE METHODS
    /////////////////////////////////
    /////////////////////////////////
    // STEP 1
    /////////////////////////////////
    /**
     * Spawn vehicles.
     *
     * @param timeStep the time step
     */
    private void spawnVehicles(double timeStep) {
        if (DesignatedLanesExpr.exprType == DesignatedLanesExpr.ExprType.DESIGNATED_LANES) {
            DesignatedLanesExpr.spawnVehicles(timeStep, this);
        } else if (DesignatedLanesExpr.exprType == DesignatedLanesExpr.ExprType.DESIGNATED_LANES_WITH_ARCH) {
            DesignatedLanesExpr.spawnVehiclesWithInterArch(timeStep, this);
        } else {
            for (SpawnPoint spawnPoint : basicMap.getSpawnPoints()) {
                // figure out whether it can spawn - now vehicles too near
                List<SpawnSpec> spawnSpecs = spawnPoint.act(timeStep);
                if (!spawnSpecs.isEmpty()) {
                    for (SpawnSpec spawnSpec : spawnSpecs) {
                        if (canSpawnVehicle(spawnPoint)) {
                            VehicleSimView vehicle = makeVehicle(spawnPoint, spawnSpec);
                            VinRegistry.registerVehicle(vehicle); // Get vehicle a VIN number

                            vinToVehicles.put(vehicle.getVIN(), vehicle);
                            spawnPoint.vehicleGenerated(); // so it knows a platooning vehicle is generated.

                            generatedVehicles++; // counter for vehicles generated
                        }
                    }
                }
            }
        }
    }

    public int getProhibitedVehiclesNum() {
        return inhibitedVehicles;
    }

    public int getGeneratedVehiclesNum() {
        return generatedVehicles;
    }

    /**
     * Whether a spawn point can spawn any vehicle
     *
     * @param spawnPoint the spawn point
     * @return Whether the spawn point can spawn any vehicle
     */
    public boolean canSpawnVehicle(SpawnPoint spawnPoint) {
        // TODO: can be made much faster.
        Rectangle2D noVehicleZone = spawnPoint.getNoVehicleZone();
        for (VehicleSimView vehicle : vinToVehicles.values()) {
            if (vehicle.getShape().intersects(noVehicleZone)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Create a vehicle at a spawn point.
     *
     * @param spawnPoint the spawn point
     * @param spawnSpec the spawn specification
     * @return the vehicle
     */
    public VehicleSimView makeVehicle(SpawnPoint spawnPoint,
            SpawnSpec spawnSpec) {
        VehicleSpec spec = spawnSpec.getVehicleSpec();
        Lane lane = spawnPoint.getLane();
        // Now just take the minimum of the max velocity of the vehicle, and
        // the speed limit in the lane
        double initVelocity = Math.min(spec.getMaxVelocity(), lane.getSpeedLimit());
        // Obtain a Vehicle
        AutoVehicleSimView vehicle
                = new BasicAutoVehicle(spec,
                        spawnPoint.getPosition(),
                        spawnPoint.getHeading(),
                        spawnPoint.getSteeringAngle(),
                        initVelocity, // velocity
                        initVelocity, // target velocity
                        spawnPoint.getAcceleration(),
                        spawnSpec.getSpawnTime(),
                        spawnSpec.getVehicleType());
        // Set the driver
        AutoDriver driver = new AutoDriver(vehicle, basicMap);
        driver.setCurrentLane(lane);
        driver.setSpawnPoint(spawnPoint);
        driver.setDestination(spawnSpec.getDestinationRoad());
        vehicle.setDriver(driver);

        return vehicle;
    }

    /////////////////////////////////
    // STEP 2
    /////////////////////////////////
    /**
     * Compute the lists of vehicles of all lanes.
     *
     * @return a mapping from lanes to lists of vehicles sorted by their
     * distance on their lanes
     */
    private Map<Lane, SortedMap<Double, VehicleSimView>> computeVehicleLists() {
        // Set up the structure that will hold all the Vehicles as they are
        // currently ordered in the Lanes
        Map<Lane, SortedMap<Double, VehicleSimView>> vehicleLists
                = new HashMap<Lane, SortedMap<Double, VehicleSimView>>();
        for (Road road : basicMap.getRoads()) {
            for (Lane lane : road.getLanes()) {
                vehicleLists.put(lane, new TreeMap<Double, VehicleSimView>());
            }
        }
        // Now add each of the Vehicles, but make sure to exclude those that are
        // already inside (partially or entirely) the intersection
        for (VehicleSimView vehicle : vinToVehicles.values()) {
            // Find out what lanes it is in.
            Set<Lane> lanes = vehicle.getDriver().getCurrentlyOccupiedLanes();
            for (Lane lane : lanes) {
                // Find out what IntersectionManager is coming up for this vehicle
                IntersectionManager im
                        = lane.getLaneIM().nextIntersectionManager(vehicle.getPosition());
                // Only include this Vehicle if it is not in the intersection.
                if (true || lane.getLaneIM().distanceToNextIntersection(vehicle.getPosition()) > 0
                        || im == null || !im.intersects(vehicle.getShape().getBounds2D())) {
                    // Now find how far along the lane it is.
                    double dst = lane.distanceAlongLane(vehicle.getPosition());
                    // Now add it to the map.
                    vehicleLists.get(lane).put(dst, vehicle);
                }
                lane.getLaneIM().setVehiclesInLane(Collections.unmodifiableSortedMap(vehicleLists.get(lane)));
            }
        }

        return vehicleLists;
    }

    //todo, check if something more effecient can be done than a copy of vehicleLists first
    /**
     * Consolidates vehicleList into a new list by connected lanes for purposes
     * of later mapping nth vehicle in lane to (n+1)th vehicle
     *
     * @return a mapping from lanes to lists of vehicles sorted by their
     * distance on their lanes with connecting lanes consolidated.
     */
    private Map<Lane, SortedMap<Double, VehicleSimView>> consolidateVehicleList(Map<Lane, SortedMap<Double, VehicleSimView>> vehicleLists) {
        Map<Lane, SortedMap<Double, VehicleSimView>> consolVehicleLists
                = new HashMap<Lane, SortedMap<Double, VehicleSimView>>(vehicleLists);
        // Now consolidate the lists based on lanes
        for (Road road : basicMap.getRoads()) {
            for (Lane lane : road.getLanes()) {
                // We may have already removed this Lane from the map
                if (consolVehicleLists.containsKey(lane)) {
                    Lane currLane = lane;
                    // Now run through the lanes
                    while (currLane.hasNextLane()) {
                        currLane = currLane.getNextLane();
                        // Put everything from the next lane into the original lane
                        // and remove the mapping for the next lane
                        consolVehicleLists.get(lane).putAll(consolVehicleLists.remove(currLane));
                    }
                }
            }
        }
        return consolVehicleLists;
    }

    /**
     * Compute the next vehicles of all vehicles.
     *
     * @param vehicleLists a mapping from lanes to lists of vehicles sorted by
     * their distance on their lanes
     * @return a mapping from vehicles to next vehicles
     */
    private Map<VehicleSimView, VehicleSimView> computeNextVehicle(
            Map<Lane, SortedMap<Double, VehicleSimView>> vehicleLists) {
        // At this point we should only have mappings for start Lanes, and they
        // should include all the Lanes they run into.  Now we need to turn this
        // into a hash map that maps Vehicles to the next vehicle in the Lane
        // or any Lane the Lane runs into
        Map<VehicleSimView, VehicleSimView> nextVehicle
                = new HashMap<VehicleSimView, VehicleSimView>();
        // For each of the ordered lists of vehicles, each lane
        for (SortedMap<Double, VehicleSimView> vehicleList : vehicleLists.values()) {
            VehicleSimView lastVehicle = null;
            // Go through the Vehicles in order of their position in the Lane
            for (VehicleSimView currVehicle : vehicleList.values()) {
                if (lastVehicle != null) {
                    // Create the mapping from the previous Vehicle to the current one
                    nextVehicle.put(lastVehicle, currVehicle);

                    lastVehicle.setFrontVehicle(currVehicle);
                }

                lastVehicle = currVehicle;
            }
        }

        return nextVehicle;
    }

    /**
     * Provide each vehicle with sensor information to allow it to make
     * decisions. This works first by making an ordered list for each Lane of
     * all the vehicles in that Lane, in order from the start of the Lane to the
     * end of the Lane. We must make sure to leave out all vehicles that are in
     * the intersection. We must also concatenate the lists for lanes that feed
     * into one another. Then, for each vehicle, depending on the state of its
     * sensors, we provide it with the appropriate sensor input.
     */
    private void provideSensorInput() {
        Map<Lane, SortedMap<Double, VehicleSimView>> vehicleLists
                = computeVehicleLists();
        Map<Lane, SortedMap<Double, VehicleSimView>> consolVecList = consolidateVehicleList(vehicleLists);
        Map<VehicleSimView, VehicleSimView> nextVehicle
                = computeNextVehicle(consolVecList);

        provideIntervalInfo(nextVehicle);
        provideVehicleTrackingInfo(consolVecList, vehicleLists);
        provideTrafficSignal();
    }

    // for debug
    /*
         for(SortedMap<Double,VehicleSimView> vehicleList : vehicleLists.values()) {
         for(VehicleSimView currVehicle : vehicleList.values()) {
         VehicleSimView frontVehicle = currVehicle.getFrontVehicle();
      	
         System.out.print(currVehicle.getVIN());
      	
         if (frontVehicle == null) {
         System.out.println("Nothing");
         }
         else {
         System.out.println(frontVehicle.getVIN());
         }
         }
         }
     */
    /**
     * Provide sensing information to the intervalometers of all vehicles.
     *
     * @param nextVehicle a mapping from vehicles to next vehicles
     */
    private void provideIntervalInfo(
            Map<VehicleSimView, VehicleSimView> nextVehicle) {

        // Now that we have this list set up, let's provide input to all the
        // Vehicles.
        for (VehicleSimView vehicle : vinToVehicles.values()) {
            // If the vehicle is autonomous
            if (vehicle instanceof AutoVehicleSimView) {
                AutoVehicleSimView autoVehicle = (AutoVehicleSimView) vehicle;

                switch (autoVehicle.getLRFMode()) {
                    case DISABLED:
                        // Find the interval to the next vehicle
                        double interval;
                        // If there is a next vehicle, then calculate it
                        if (nextVehicle.containsKey(autoVehicle)) {
                            // It's the distance from the front of this Vehicle to the point
                            // at the rear of the Vehicle in front of it
                            interval = calcInterval(autoVehicle, nextVehicle.get(autoVehicle));
                        } else { // Otherwise, just set it to the maximum possible value
                            interval = Double.MAX_VALUE;
                        }
                        // Now actually record it in the vehicle
                        autoVehicle.getIntervalometer().record(interval);
                        autoVehicle.setLRFSensing(false); // Vehicle is not using
                        // the LRF sensor
                        break;
                    case LIMITED:
                        // FIXME
                        autoVehicle.setLRFSensing(true); // Vehicle is using the LRF sensor
                        break;
                    case ENABLED:
                        // FIXME
                        autoVehicle.setLRFSensing(true); // Vehicle is using the LRF sensor
                        break;
                    default:
                        throw new RuntimeException("Unknown LRF Mode: "
                                + autoVehicle.getLRFMode().toString());
                }
            }
        }
    }

    /**
     * Provide tracking information to vehicles.
     *
     * @param consolidatedVehicleLists a mapping from lanes to lists of vehicles
     * sorted by their distance on their lanes
     * @param vehicleListsByLane a mapping of lanes to vehicles based on
     * position in lane
     */
    private void provideVehicleTrackingInfo(
            Map<Lane, SortedMap<Double, VehicleSimView>> consolidatedVehicleLists, Map<Lane, SortedMap<Double, VehicleSimView>> vehicleListsByLane) {
        // Vehicle Tracking
        for (VehicleSimView vehicle : vinToVehicles.values()) {
            // If the vehicle is autonomous
            if (vehicle instanceof AutoVehicleSimView) {
                AutoVehicleSimView autoVehicle = (AutoVehicleSimView) vehicle;

                // vehicle tracking is set to be false. Nothing run here in this if clause
                if (autoVehicle.isVehicleTracking()) {
                    DriverSimView driver = autoVehicle.getDriver();
                    Lane targetLane = autoVehicle.getTargetLaneForVehicleTracking();
                    Point2D pos = autoVehicle.getPosition();
                    double dst = targetLane.distanceAlongLane(pos);

                    // initialize the distances to infinity
                    double frontDst = Double.MAX_VALUE;
                    double rearDst = Double.MAX_VALUE;
                    VehicleSimView frontVehicle = null;
                    VehicleSimView rearVehicle = null;

                    // only consider the vehicles on the target lane
                    SortedMap<Double, VehicleSimView> vehiclesOnTargetLane
                            = consolidatedVehicleLists.get(targetLane);

                    // compute the distances and the corresponding vehicles
                    try {
                        double d = vehiclesOnTargetLane.tailMap(dst).firstKey();
                        frontVehicle = vehiclesOnTargetLane.get(d);
                        frontDst = (d - dst) - frontVehicle.getSpec().getLength();
                    } catch (NoSuchElementException e) {
                        frontDst = Double.MAX_VALUE;
                        frontVehicle = null;
                    }
                    try {
                        double d = vehiclesOnTargetLane.headMap(dst).lastKey();
                        rearVehicle = vehiclesOnTargetLane.get(d);
                        rearDst = dst - d;
                    } catch (NoSuchElementException e) {
                        rearDst = Double.MAX_VALUE;
                        rearVehicle = null;
                    }

                    // assign the sensor readings
                    autoVehicle.getFrontVehicleDistanceSensor().record(frontDst);
                    autoVehicle.getRearVehicleDistanceSensor().record(rearDst);

                    // assign the vehicles' velocities
                    if (frontVehicle != null) {
                        autoVehicle.getFrontVehicleSpeedSensor().record(
                                frontVehicle.getVelocity());
                    } else {
                        autoVehicle.getFrontVehicleSpeedSensor().record(Double.MAX_VALUE);
                    }
                    if (rearVehicle != null) {
                        autoVehicle.getRearVehicleSpeedSensor().record(
                                rearVehicle.getVelocity());
                    } else {
                        autoVehicle.getRearVehicleSpeedSensor().record(Double.MAX_VALUE);
                    }

                    // show the section on the viewer
                    if (Debug.isTargetVIN(driver.getVehicle().getVIN())) {
                        Point2D p1 = targetLane.getPointAtNormalizedDistance(
                                Math.max((dst - rearDst) / targetLane.getLength(), 0.0));
                        Point2D p2 = targetLane.getPointAtNormalizedDistance(
                                Math.min((frontDst + dst) / targetLane.getLength(), 1.0));
                        Debug.addLongTermDebugPoint(
                                new DebugPoint(p2, p1, "cl", Color.RED.brighter()));
                    }
                }

                // we need each vehicle to keep the vehicle before it
            }
        }

    }

    /**
     * Provide traffic signals.
     */
    private void provideTrafficSignal() {
        for (VehicleSimView vehicle : vinToVehicles.values()) {
            if (vehicle instanceof HumanDrivenVehicleSimView) {
                HumanDrivenVehicleSimView manualVehicle
                        = (HumanDrivenVehicleSimView) vehicle;
                provideTrafficLightSignal(manualVehicle);
            }
        }
    }

    /**
     * Calculate the distance between vehicle and the next vehicle on a lane.
     *
     * @param vehicle the vehicle
     * @param nextVehicle the next vehicle
     * @return the distance between vehicle and the next vehicle on a lane
     */
    private double calcInterval(VehicleSimView vehicle,
            VehicleSimView nextVehicle) {
        // From Chiu: Kurt, if you think this function is not okay, probably
        // we should talk to see what to do.
        /*Point2D pos = vehicle.getPosition();
        if (nextVehicle.getShape().contains(pos)) {
            return 0.0;
        } else {
            // TODO: make it more efficient
            double interval = Double.MAX_VALUE;
            for (Line2D edge : nextVehicle.getEdges()) {
                double dst = edge.ptSegDist(pos);
                if (dst < interval) {
                    interval = dst;
                }
            }
            return interval;
        }*/
        Point2D pos = vehicle.getPosition();
        if (nextVehicle.getShape().contains(pos)) {
            return 0.0;
        } else {
            // TODO: make it more efficient
            double interval = Double.MAX_VALUE;
            for (Line2D edge : nextVehicle.getEdges()) {
                double dst = edge.ptSegDist(pos);
                if (dst < interval) {
                    interval = dst;
                }
            }
            return interval;
        }

        //todo, this is still a problem, but seems to be preexisting even with this change below, so this was reverted to the old code until more investigation can be done
        /* double interval = Double.MAX_VALUE;
            double dst = Double.MAX_VALUE;
            for (Line2D nextVehicleEdge : nextVehicle.getEdges()) {
                for (Line2D vehicleEdge : vehicle.getEdges()) {
                    if (nextVehicleEdge.intersectsLine(vehicleEdge)) {
                        throw new RuntimeException("When attempting to calculate distance between vehicles for sensor updates, two vehicles were calculated to have collided.");
                    }
                    //check all endpoints of both line segments to the other line segment
                    dst = Math.min(dst, Math.min(nextVehicleEdge.ptSegDist(vehicleEdge.getP1()), nextVehicleEdge.ptSegDist(vehicleEdge.getP2())));
                    dst = Math.min(dst, Math.min(vehicleEdge.ptSegDist(nextVehicleEdge.getP1()), vehicleEdge.ptSegDist(nextVehicleEdge.getP2())));
                    if (dst < interval) {
                        interval = dst;
                    }
                }
            }
            return interval;
        }*/
    }
    // Kurt's code:
    // interval = vehicle.getPosition().
    //   distance(nextVehicle.get(vehicle).getPointAtRear());

    /**
     * Provide traffic light signals to a vehicle.
     *
     * @param vehicle the vehicle
     */
    private void provideTrafficLightSignal(HumanDrivenVehicleSimView vehicle) {
        // TODO: implement it later
//    DriverSimView driver = vehicle.getDriver();
//    Lane currentLane = driver.getCurrentLane();
//    Point2D pos = vehicle.getPosition();
//    IntersectionManager im = currentLane.getLaneIM().
//                             nextIntersectionManager(pos);
//    if (im != null) {
//      if (im instanceof LightOnlyManager) {
//        LightOnlyManager lightOnlyIM = (LightOnlyManager)im;
//        if (!im.getIntersection().getArea().contains(pos)) {
//          LightState s = lightOnlyIM.getLightState(currentLane);
//          vehicle.setLightState(s);
//          if (driver instanceof HumanDriver) {
//            ((HumanDriver)driver).setLightState(s);
//          }
//        } else {
//          vehicle.setLightState(null);
//          if (driver instanceof HumanDriver) {
//            ((HumanDriver)driver).setLightState(null);
//          }
//        }
//      }
//    } else {
//      vehicle.setLightState(null);
//      if (driver instanceof HumanDriver) {
//        ((HumanDriver)driver).setLightState(null);
//      }
//    }
    }

    /////////////////////////////////
    // STEP 3
    /////////////////////////////////
    /**
     * Allow each driver to act.
     */
    private void letDriversAct() {
        for (VehicleSimView vehicle : vinToVehicles.values()) {
            vehicle.getDriver().act();

            if (Resources.vinToLane.get(vehicle.getVIN()) != null) {
                if (Resources.vinToLane.get(vehicle.getVIN()) != vehicle.getDriver().getCurrentLane()) {
                    Resources.laneToVin.get(Resources.vinToLane.get(vehicle.getVIN())).remove(vehicle.getVIN());
                }
            }
            Resources.laneToVin.get(vehicle.getDriver().getCurrentLane()).add(vehicle.getVIN());
            Resources.vinToLane.put(vehicle.getVIN(), vehicle.getDriver().getCurrentLane());

            if (vehicle.getDriver().getState() == State.V2I_TRAVERSING) {
                Resources.traversingVehicles.add(vehicle.getVIN());
            } else if (vehicle.getDriver().getState() == State.V2I_CLEARING) {
                Resources.traversingVehicles.remove(vehicle.getVIN());
            }
        }
    }

    /////////////////////////////////
    // STEP 4
    /////////////////////////////////
    /**
     * Allow each intersection manager to act.
     *
     * @param timeStep the time step
     */
    private void letIntersectionManagersAct(double timeStep) {
        for (IntersectionManager im : basicMap.getIntersectionManagers()) {
            im.act(timeStep);
        }
    }

    /////////////////////////////////
    // STEP 5
    /////////////////////////////////
    /**
     * Deliver the V2I and I2V messages.
     */
    private void communication() {
        deliverV2IMessages();
        deliverI2VMessages();
//    deliverV2VMessages();
    }

    /**
     * Deliver the V2I messages.
     */
    private void deliverV2IMessages() {
        // Go through each vehicle and deliver each of its messages
        for (VehicleSimView vehicle : vinToVehicles.values()) {
            // Start with V2I messages
            if (vehicle instanceof AutoVehicleSimView) {
                AutoVehicleSimView sender = (AutoVehicleSimView) vehicle;
                Queue<V2IMessage> v2iOutbox = sender.getV2IOutbox();
                while (!v2iOutbox.isEmpty()) {
                    V2IMessage msg = v2iOutbox.poll();
                    V2IManager receiver
                            = (V2IManager) basicMap.getImRegistry().get(msg.getImId());
                    // Calculate the distance the message must travel
                    double txDistance
                            = sender.getPosition().distance(
                                    receiver.getIntersection().getCentroid());
                    // Find out if the message will make it that far
                    if (transmit(txDistance, sender.getTransmissionPower())) {
                        // Actually deliver the message
                        receiver.receive(msg);
                        // Add the delivery to the debugging information
                    }
                    // Either way, we increment the number of transmitted messages
                }
            }
        }
    }

    /**
     * Deliver the I2V messages.
     */
    private void deliverI2VMessages() {
        // Now deliver all the I2V messages
        for (IntersectionManager im : basicMap.getIntersectionManagers()) {
            V2IManager senderIM = (V2IManager) im;
            for (Iterator<I2VMessage> i2vIter = senderIM.outboxIterator();
                    i2vIter.hasNext();) {
                I2VMessage msg = i2vIter.next();
                AutoVehicleSimView vehicle
                        = (AutoVehicleSimView) VinRegistry.getVehicleFromVIN(
                                msg.getVin());
                // Calculate the distance the message must travel
                double txDistance
                        = senderIM.getIntersection().getCentroid().distance(
                                vehicle.getPosition());
                // Find out if the message will make it that far
                if (transmit(txDistance, senderIM.getTransmissionPower())) {
                    // Actually deliver the message
                    vehicle.receive(msg);
                }
            }

            // Done delivering the IntersectionManager's messages, so clear the
            // outbox.
            senderIM.clearOutbox();
        }
    }

//  private void deliverV2VMessages() {
//
//    // Create a place to store broadcast messages until they can be sent so
//    // that we don't have to run through the list of Vehicles for each one
//    List<V2VMessage> broadcastMessages = new ArrayList<V2VMessage>();
//
//    // Go through each vehicle and deliver each of its messages
//    for(VehicleSimView vehicle : vinToVehicles.values()) {
//      // Then do V2V messages.
//      if (vehicle instanceof AutoVehicleSimView) {
//        AutoVehicleSimView sender = (AutoVehicleSimView)vehicle;
//        for(V2VMessage msg : sender.getV2VOutbox()) {
//          if(msg.isBroadcast()) {
//            // If it's a broadcast message, we save it and worry about it later
//            broadcastMessages.add(msg);
//          } else {
//            // Otherwise, we just deliver it! Woo!
//            // TODO: need to check whether the vehicle is AutoVehicleSpec
//            AutoVehicleSimView receiver =
//              (AutoVehicleSimView)VinRegistry.getVehicleFromVIN(
//                msg.getDestinationID());
//            // Calculate the distance the message must travel
//            double txDistance =
//              sender.getPosition().distance(receiver.getPosition());
//            // Find out if the message will make it that far
//            if(transmit(txDistance, sender.getTransmissionPower())) {
//              // Actually deliver the message
//              receiver.receive(msg);
//              // Add the delivery to the debugging information
//            }
//          }
//        }
//        // Done delivering the V2V messages (except broadcast which we will
//        // handle in a moment), so clear the outbox
//        sender.getV2VOutbox().clear();
//      }
//    }
//    // Now go through the vehicles and deliver the broadcast messages
//    for(V2VMessage msg : broadcastMessages) {
//      // Send a copy to the IM for debugging/statistics purposes
//      IntersectionManager im =
//        basicMap.getImRegistry().get(msg.getIntersectionManagerID());
////      if(im != null) {
////        switch(im.getIntersectionType()) {
////        case V2V:
////          ((V2VManager) im).logBroadcast(msg);
////        }
////      }
//      // Determine who sent this message
//      // TODO: need to check whether the vehicle is AutoVehicleSpec
//      AutoVehicleSimView sender =
//        (AutoVehicleSimView)VinRegistry.getVehicleFromVIN(
//          msg.getSourceID());
//      // Deliver to each vehicle
//      for(VehicleSimView vehicle : vinToVehicles.values()) {
//        if (vehicle instanceof AutoVehicleSimView) {
//          AutoVehicleSimView receiver = (AutoVehicleSimView)vehicle;
//          // Except the one that sent it
//          if(sender != receiver) {
//            // Find out how far away they are
//            double txDistance =
//              sender.getPosition().distance(receiver.getPosition());
//            // See if this Vehicle is close enough to receive the message
//            if(transmit(txDistance, sender.getTransmissionPower())) {
//              // Actually deliver the message
//              receiver.receive(msg);
//            }
//          }
//        } // else ignore other non-autonomous vehicle
//      }
//    }
//  }
    /**
     * Whether the transmission of a message is successful
     *
     * @param distance the distance of the transmission
     * @param power the power of the transmission
     * @return whether the transmission of a messsage is successful
     */
    private boolean transmit(double distance, double power) {
        // Simple for now
        return distance <= power;
    }

    /////////////////////////////////
    // STEP 6
    /////////////////////////////////
    /**
     * Move all the vehicles.
     *
     * @param timeStep the time step
     */
    private void moveVehicles(double timeStep) {
        // calculate vehicles inside the intersection
        int vehiclesInside = 0;

        for (VehicleSimView vehicle : vinToVehicles.values()) {
            Point2D p1 = vehicle.getPosition();
            vehicle.move(timeStep);
            Point2D p2 = vehicle.getPosition();

            if (p1.distance(p2) < 0.001) {
                vehicle.askedToStop();
            }

            // if this vehicle is in the intersection, judged by DCL,
            // vehiclesInside++, it's the counter
            // TODO not understanding where to get intersection boundary data!!
            if (p2.getX() >= 149.5 && p2.getX() <= 175.5
                    && p2.getY() >= 149.5 && p2.getY() <= 175.5) {
                vehiclesInside++;
            }

//            for (Road road : basicMap.getRoads()) {
//                for(Lane lane : road.getLanes())
//                link.intersect(vehicle, currentTime, p1, p2);
//            }
            for (DataCollectionLine line : basicMap.getDataCollectionLines()) {
                line.intersect(vehicle, currentTime, p1, p2);
            }
            if (Debug.isPrintVehicleStateOfVIN(vehicle.getVIN())) {
                vehicle.printState();
            }
        }

        /*
         // output only when it's x.00 sec
         if (currentTime % 0.1 < 0.001 || currentTime % 0.1 > 0.099) {
         System.out.printf("%f,%d\n", currentTime, vehiclesInside);
         }
         */
    }

    /////////////////////////////////
    // STEP 7
    /////////////////////////////////
    /**
     * Remove all completed vehicles.
     *
     * @return the VINs of the completed vehicles
     */
    private List<Integer> cleanUpCompletedVehicles() {
        List<Integer> completedVINs = new LinkedList<Integer>();

        Rectangle2D mapBoundary = basicMap.getDimensions();

        List<Integer> removedVINs = new ArrayList<Integer>(vinToVehicles.size());
        for (int vin : vinToVehicles.keySet()) {
            VehicleSimView v = vinToVehicles.get(vin);

            // If the vehicle is no longer in the layout
            // TODO: this should be replaced with destination zone.
            if (!v.getShape().intersects(mapBoundary)) {
                // Process all the things we need to from this vehicle
                if (v instanceof AutoVehicleSimView) {
                    AutoVehicleSimView v2 = (AutoVehicleSimView) v;
                    totalBitsTransmittedByCompletedVehicles += v2.getBitsTransmitted();
                    totalBitsReceivedByCompletedVehicles += v2.getBitsReceived();
                }
                removedVINs.add(vin);
            }
        }
        // Remove the marked vehicles
        for (int vin : removedVINs) {

            completedVINs.add(vin);

            Resources.laneToVin.get(Resources.vinToLane.get(vin)).remove(vin);
            Resources.vinToLane.remove(vin);

            VehicleSimView v = vinToVehicles.get(vin);
            Driver vehicleDriver = v.getDriver();
            double travelTime = currentTime - v.getSpawnTime();
            double lastMax;
            //todo, I'm committing a sin with regard to this inheritance/instance of checking, but there's something weird going on with VINs and initial speed limits at the Driver class level (vins start at -1 a lot of times and the vehicle is acted on, and they have a speed less than the limit on spawn), so that's why I did it at the higher level
            Double delay = (vehicleDriver instanceof AutoDriver ? ((AutoDriver) vehicleDriver).getDelayPerTimeStep() * SimConfig.TIME_STEP : null);
            Double delayNoInter = (vehicleDriver instanceof AutoDriver ? ((AutoDriver) vehicleDriver).getDelayPerTimeStepExcludingIntersections() * SimConfig.TIME_STEP : null);

            if (v.isHuman()) {
                TrafficSignalExpr.Htotal++;
                TrafficSignalExpr.HtotalTime += travelTime;
                lastMax = DesignatedLanesExpr.maxTravelTimeH;
                DesignatedLanesExpr.maxTravelTimeH = Math.max(travelTime, DesignatedLanesExpr.maxTravelTimeH);
                if (shouldUpdateTime(lastMax, DesignatedLanesExpr.maxTravelTimeH)) {
                    DesignatedLanesExpr.timeMaxTravelHCompletion = currentTime;
                }
                DesignatedLanesExpr.humanVehicleTimesByDirection.get(DesignatedLanesExpr.vinToTimeIndex.get(vin)).get(DesignatedLanesExpr.timingIndexMap.get(DesignatedLanesExpr.vinToSpawnDirection.get(vin))).add(travelTime);
                if (delay != null && delayNoInter != null) {
                    DesignatedLanesExpr.humanVehicleDelaysByDirection.get(DesignatedLanesExpr.vinToTimeIndex.get(vin)).get(DesignatedLanesExpr.timingIndexMap.get(DesignatedLanesExpr.vinToSpawnDirection.get(vin))).add(delay);
                    DesignatedLanesExpr.humanVehicleDelaysByDirectionNoInter.get(DesignatedLanesExpr.vinToTimeIndex.get(vin)).get(DesignatedLanesExpr.timingIndexMap.get(DesignatedLanesExpr.vinToSpawnDirection.get(vin))).add(delayNoInter);
                }
                if (vehicleDriver instanceof AutoDriver && !((AutoDriver) vehicleDriver).getWasStopped()) {
                    DesignatedLanesExpr.humanVehicleDelaysExcludingStops.get(DesignatedLanesExpr.vinToTimeIndex.get(vin)).get(DesignatedLanesExpr.timingIndexMap.get(DesignatedLanesExpr.vinToSpawnDirection.get(vin))).add(delayNoInter);
                }
            } else if (v.withAdaptiveCruiseControll()) {
                TrafficSignalExpr.SAVtotal++;
                TrafficSignalExpr.SAVtotalTime += travelTime;
                lastMax = DesignatedLanesExpr.maxTravelTimeACC;
                DesignatedLanesExpr.maxTravelTimeACC = Math.max(travelTime, DesignatedLanesExpr.maxTravelTimeACC);
                if (shouldUpdateTime(lastMax, DesignatedLanesExpr.maxTravelTimeACC)) {
                    DesignatedLanesExpr.timeMaxTravelACCCompletion = currentTime;
                }
            } else if (v.withCruiseControll()) {
                TrafficSignalExpr.SAVtotal++;
                TrafficSignalExpr.SAVtotalTime += travelTime;
                lastMax = DesignatedLanesExpr.maxTravelTimeCC;
                DesignatedLanesExpr.maxTravelTimeCC = Math.max(travelTime, DesignatedLanesExpr.maxTravelTimeCC);
                if (shouldUpdateTime(lastMax, DesignatedLanesExpr.maxTravelTimeCC)) {
                    DesignatedLanesExpr.timeMaxTravelCCCompletion = currentTime;
                }
            } else {
                TrafficSignalExpr.AVtotal++;
                TrafficSignalExpr.AVtotalTime += travelTime;
                lastMax = DesignatedLanesExpr.maxTravelTimeAV;
                DesignatedLanesExpr.maxTravelTimeAV = Math.max(travelTime, DesignatedLanesExpr.maxTravelTimeAV);
                if (shouldUpdateTime(lastMax, DesignatedLanesExpr.maxTravelTimeAV)) {
                    DesignatedLanesExpr.timeMaxTravelAVCompletion = currentTime;
                }
                DesignatedLanesExpr.autoVehicleTimesByDirection.get(DesignatedLanesExpr.vinToTimeIndex.get(vin)).get(DesignatedLanesExpr.timingIndexMap.get(DesignatedLanesExpr.vinToSpawnDirection.get(vin))).add(travelTime);
                if (delay != null && delayNoInter != null) {
                    DesignatedLanesExpr.autoVehicleDelaysByDirection.get(DesignatedLanesExpr.vinToTimeIndex.get(vin)).get(DesignatedLanesExpr.timingIndexMap.get(DesignatedLanesExpr.vinToSpawnDirection.get(vin))).add(delay);
                    DesignatedLanesExpr.autoVehicleDelaysByDirectionNoInter.get(DesignatedLanesExpr.vinToTimeIndex.get(vin)).get(DesignatedLanesExpr.timingIndexMap.get(DesignatedLanesExpr.vinToSpawnDirection.get(vin))).add(delayNoInter);
                }
                if (vehicleDriver instanceof AutoDriver && !((AutoDriver) vehicleDriver).getWasStopped()) {
                    DesignatedLanesExpr.autoVehicleDelaysExcludingStops.get(DesignatedLanesExpr.vinToTimeIndex.get(vin)).get(DesignatedLanesExpr.timingIndexMap.get(DesignatedLanesExpr.vinToSpawnDirection.get(vin))).add(delayNoInter);
                }
            }

            completionTimes.add(travelTime);
            numOfCompletedVehicles++;
            vinToVehicles.remove(vin);
            DesignatedLanesExpr.vinToTimeIndex.remove(vin);
            DesignatedLanesExpr.vinToSpawnDirection.remove(vin);
        }

        return completedVINs;
    }

    //compares a previous double and a current double (representing max times), if they don't equal returns true
    private boolean shouldUpdateTime(double prev, double current) {
        return prev != current;
    }

    /////////////////////////////////
    // DEBUG
    /////////////////////////////////
    /**
     * Check whether the clocks are in sync.
     */
    private void checkClocks() {
        // Check the clocks for all autonomous vehicles.
        for (VehicleSimView vehicle : vinToVehicles.values()) {
            vehicle.checkCurrentTime(currentTime);
        }
        // Check the clocks for all the intersection managers.
        for (IntersectionManager im : basicMap.getIntersectionManagers()) {
            im.checkCurrentTime(currentTime);
        }
    }

    /////////////////////////////////
    // PUBLIC METHODS
    /////////////////////////////////
    // information retrieval
    /**
     * If the current time exceeds the end of the total duration of traffic
     * lights, re-select the proper red signal time.
     */
    //used for red phase adaptive
    private void updateTrafficSignal() {
        // get the list of signal controllers
        double tl = GridMapUtil.getTrafficLevel(); // traffic level
        double hp = SimConfig.HUMAN_PERCENTAGE; // human percentage

        double rp = RedPhaseData.getRedPhase(hp, tl); // red phase
        double offset = ApproxNPhasesTrafficSignalRequestHandler.CyclicSignalController.getEndTime();

        RingAndBarrier ringAndBarrier = Resources.ringAndBarrier;
        Map<Integer, SignalController> signalControllers = Resources.signalControllers;

        ringAndBarrier.LEGACY_resetRedDurations(rp);
        System.out.printf("Appropriate Red Phase Length: %f\n", rp);

        for (Road road : Resources.im.getIntersection().getEntryRoads()) {
            for (Lane lane : road.getLanes()) {
                CyclicSignalController controller
                        = ringAndBarrier.LEGACY_calcCyclicSignalController(road);
                controller.setOffset(offset);

                signalControllers.put(lane.getId(), controller);
            }
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public synchronized double getAvgTravelTime() {
        double ans = 0;
        for (double time : completionTimes) {
            ans += time;
        }
        return ans / completionTimes.size();
    }

    @Override
    public synchronized double getTimeSTD() {
        double ans = 0;
        double avg = getAvgTravelTime();
        for (Double t : completionTimes) {
            ans += Math.abs(t - avg);
        }
        return ans / completionTimes.size();
    }

    @Override
    public int getScheduledVehiclesRemaining() {
        HashSet<SpawnSpecGenerator> seenGenerators = new HashSet<SpawnSpecGenerator>();
        boolean foundScheduler = false;
        int totalLeft = 0;
        for (SpawnPoint sp : basicMap.getSpawnPoints()) {
            SpawnSpecGenerator specGen = sp.getVehicleSpecChooser();
            if (!seenGenerators.contains(specGen)) {
                seenGenerators.add(specGen);
                if (specGen != null && specGen instanceof FileSpawnSpecGenerator) {
                    totalLeft += ((FileSpawnSpecGenerator) specGen).getVehiclesLeftInCurrentTimeSlot();
                    foundScheduler = true;
                }
            }
        }
        return (foundScheduler ? totalLeft : -1);
    }

    @Override
    public int getTotalScheduledVehicles() {
        HashSet<SpawnSpecGenerator> seenGenerators = new HashSet<SpawnSpecGenerator>();
        boolean foundScheduler = false;
        int totalLeft = 0;
        for (SpawnPoint sp : basicMap.getSpawnPoints()) {
            SpawnSpecGenerator specGen = sp.getVehicleSpecChooser();
            if (!seenGenerators.contains(specGen)) {
                seenGenerators.add(specGen);
                if (specGen != null && specGen instanceof FileSpawnSpecGenerator) {
                    totalLeft += ((FileSpawnSpecGenerator) specGen).getTotalVehiclesScheduled();
                    foundScheduler = true;
                }
            }
        }
        return (foundScheduler ? totalLeft : -1);
    }

}
