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
package aim4.map;

import aim4.config.Constants.TurnDirection;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import aim4.config.Debug;
import aim4.config.OneLaneTimeConfig;
import aim4.config.Platoon;
import aim4.config.RedPhaseData;
import aim4.config.Resources;
import aim4.config.RevisedPhaseConfig;
import aim4.config.SimConfig;
import aim4.config.SimConfig.SIGNAL_TYPE;
import aim4.config.SimConfig.VEHICLE_TYPE;
import aim4.config.ringbarrier.RingAndBarrier;
import aim4.config.ringbarrier.RingAndBarrierFactory;
import aim4.im.DedicatedTrafficController;
import aim4.im.Intersection;
import aim4.im.IntersectionManager;
import aim4.im.LaneTrafficController;
import aim4.im.LaneTrafficController.LaneInfo;
import aim4.im.LaneTrafficController.SpawnCase;
import aim4.im.NormalTrafficController;
import aim4.im.RoadBasedIntersection;
import aim4.im.RoadBasedTrackModel;
import aim4.im.intersectionarch.ArchIntersection;
import aim4.im.v2i.RequestHandler.ApproxSimpleTrafficSignalRequestHandler;
import aim4.im.v2i.V2IManager;
import aim4.im.v2i.RequestHandler.ApproxNPhasesTrafficSignalRequestHandler.DedicatedLanesSignalController;
import aim4.im.v2i.RequestHandler.ApproxNPhasesTrafficSignalRequestHandler.OneLaneSignalController;
import aim4.im.v2i.RequestHandler.ApproxNPhasesTrafficSignalRequestHandler.RevisedPhaseSignalController;
import aim4.im.v2i.RequestHandler.ApproxNPhasesTrafficSignalRequestHandler.SignalController;
import aim4.im.v2i.RequestHandler.ApproxStopSignRequestHandler;
import aim4.im.v2i.RequestHandler.Approx4PhasesTrafficSignalRequestHandler;
import aim4.im.v2i.RequestHandler.ApproxNPhasesTrafficSignalRequestHandler;
import aim4.im.v2i.RequestHandler.ApproxNPhasesTrafficSignalRequestHandler.CyclicSignalController;
import aim4.im.v2i.RequestHandler.BatchModeRequestHandler;
import aim4.im.v2i.RequestHandler.FCFSRequestHandler;
import aim4.im.v2i.RequestHandler.FullyActuatedSignalController;
import aim4.im.v2i.RequestHandler.RequestHandler;
import aim4.im.v2i.batch.RoadBasedReordering;
import aim4.im.v2i.policy.BasePolicy;
import aim4.im.v2i.reservation.ReservationGridManager;
import aim4.map.SpawnPoint.SpawnSpec;
import aim4.map.SpawnPoint.SpawnSpecGenerator;
import aim4.map.destination.DestinationSelector;
import aim4.map.destination.FileBasedDestinationSelector;
import aim4.map.destination.RandomDestinationSelector;
import aim4.map.destination.TurnBasedDestinationSelector;
import aim4.map.intersectionboard.DifferingLanesPerRoadBoard;
import aim4.map.intersectionboard.IntersectionBoard;
import aim4.map.intersectionboard.StandardIntersectionBoard;
import aim4.map.lane.Lane;
import aim4.map.lane.LaneIM;
import aim4.map.trafficbyturns.DestinationFileSpawnSpecGenerator;
import aim4.map.trafficbyturns.LaneRestrictedFileSpawnSpecGenerator;
import aim4.map.trafficbyturns.TrafficFlowReaderFactory;
import aim4.map.trafficbyturns.TurnMovements;
import aim4.sim.setup.AdaptiveTrafficSignalSuperviser;
import aim4.util.Util;
import aim4.vehicle.VehicleSpec;
import aim4.vehicle.VehicleSpecDatabase;
import java.io.File;

/**
 * The utility class for GridMap.
 */
public class GridMapUtil {

    private static IntersectionBoard roadBoard = null;
    private static double currentTrafficLevel;

    /////////////////////////////////
    // NESTED CLASSES
    /////////////////////////////////
    /**
     * The null spawn spec generator that generates nothing.
     */
    public static SpawnSpecGenerator nullSpawnSpecGenerator
            = new SpawnSpecGenerator() {
        @Override
        public List<SpawnSpec> act(SpawnPoint spawnPoint, double timeStep) {
            return new ArrayList<SpawnSpec>();
        }

        @Override
        public void vehicleGenerated() {
            // TODO Auto-generated method stub

        }

        @Override
        public SpawnSpec act(SpawnPoint spawnPoint, double timeStep, VEHICLE_TYPE vehicleType, Road destinationRoad) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }

        @Override
        public List<SpawnSpec> act(SpawnPoint spawnPoint, double timeStep, VEHICLE_TYPE vehicleType) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }
    };

    /**
     * The uniform distributed spawn spec generator.
     */
    public static class UniformSpawnSpecGenerator implements SpawnSpecGenerator {

        /**
         * The proportion of each spec
         */
        private List<Double> proportion;
        /**
         * The destination selector
         */
        private DestinationSelector destinationSelector;
        /**
         * probability of generating a vehicle in each spawn time step
         */
        private double prob;
        /**
         * how many human driven vehicles are to be generated
         */
        private int vehiclesToBeGenerated = 0;
        /**
         * This contains the traffic level of each type of vehicle.
         */
        private LaneInfo laneInfo;

        /**
         * Create an uniform spawn specification generator.
         *
         * @param trafficLevel the traffic level
         * @param destinationSelector the destination selector
         */
        public UniformSpawnSpecGenerator(double trafficLevel,
                DestinationSelector destinationSelector) {
            int n = VehicleSpecDatabase.getNumOfSpec();
            proportion = new ArrayList<Double>(n);
            double p = 1.0 / n;
            for (int i = 0; i < n; i++) {
                proportion.add(p);
            }
            this.destinationSelector = destinationSelector;
            Resources.destinationSelector = destinationSelector;

            prob = trafficLevel * SimConfig.SPAWN_TIME_STEP;
            // Cannot generate more than one vehicle in each spawn time step
            assert prob <= 1.0;
        }

        /**
         * The traffic level is not required here - can be got by laneInfo. Just
         * for compacibilty.
         *
         * @param laneInfo
         * @param trafficLevel
         * @param selector
         */
        public UniformSpawnSpecGenerator(LaneInfo laneInfo,
                double trafficLevel, DestinationSelector selector) {
            this(trafficLevel, selector);
            this.laneInfo = laneInfo;
        }

        @Override
        public List<SpawnSpec> act(SpawnPoint spawnPoint, double timeStep, VEHICLE_TYPE vehicleType) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public List<SpawnSpec> act(SpawnPoint spawnPoint, double timeStep) {
            List<SpawnSpec> result = new LinkedList<SpawnSpec>();

            double initTime = spawnPoint.getCurrentTime();
            double time = initTime;

            if (vehiclesToBeGenerated > 0) {
                // it's now possible to spawn vehicles not generated before
                int i = Util.randomIndex(proportion);
                VehicleSpec vehicleSpec = VehicleSpecDatabase.getVehicleSpecById(i);
                Road destinationRoad
                        = destinationSelector.selectDestination(spawnPoint.getLane());

                VEHICLE_TYPE vehicleType = VEHICLE_TYPE.HUMAN;

                result.add(new SpawnSpec(spawnPoint.getCurrentTime(),
                        vehicleSpec,
                        destinationRoad,
                        vehicleType));

                return result;
            }

            // running until time == initTime + timeStep
            // each time, timeStep increased by SPAWN_TIME_STEP
            while ((initTime + timeStep) - time > 0.0001) {
                SpawnCase spawnCase = laneInfo.getSpawnVehicle();
                VEHICLE_TYPE vehicleType = spawnCase.getVehicleType();
                int indexInRoad = spawnPoint.getLane().getIndexInRoad();
                if (spawnCase.vehicleSpawned()) {
                    int i = Util.randomIndex(proportion);
                    VehicleSpec vehicleSpec = VehicleSpecDatabase.getVehicleSpecById(i);
                    Road destinationRoad
                            = destinationSelector.selectDestination(spawnPoint.getLane());

                    // determine whether it's a human
                    if (vehicleType == VEHICLE_TYPE.HUMAN) {
                        // if it's platooning, we generate human drivin vehicles in groups
                        if (Platoon.platooning) {
                            // for example, if we group 5 human vehicles at one time
                            // we divide the spawning possibility by 5.
                            if (Util.RANDOM_NUM_GEN.nextDouble() < 1.0 / Platoon.vehiclesNumExpection) {
                                // okay, we generate this vehicle here, but we need to generate more vehicles
                                // when it's possible.
                                vehiclesToBeGenerated += Platoon.vehiclesNumExpection - 1;
                            } else {
                                continue;
                            }
                        }
                    }
                    //if (spawnPoint.getLane().getId() == 1) {
                    result.add(new SpawnSpec(spawnPoint.getCurrentTime(),
                            vehicleSpec,
                            destinationRoad,
                            vehicleType));
                    // }
                }

                time += SimConfig.SPAWN_TIME_STEP;
            }

            return result;
        }

        public SpawnSpec act(SpawnPoint spawnPoint, double timeStep, VEHICLE_TYPE vehicleType, Road destinationRoad) {

            SpawnCase spawnCase = new SpawnCase(vehicleType);

            int i = Util.randomIndex(proportion);
            VehicleSpec vehicleSpec = VehicleSpecDatabase.getVehicleSpecById(i);
            //if (spawnPoint.getLane().getId() == 1) {
            return new SpawnSpec(spawnPoint.getCurrentTime(),
                    vehicleSpec,
                    destinationRoad,
                    vehicleType);
            // }

        }

        /**
         * To tell this class that the platooning vehicle is successfully
         * generated. Because of the no vehicle zone, this class is not sure
         * whether his spawning is satisfied.
         */
        @Override
        public void vehicleGenerated() {
            if (vehiclesToBeGenerated > 0) {
                vehiclesToBeGenerated--;
            }
        }
    }

    /**
     * The spawn spec generator that generates only one spec.
     */
    public static class OneSpawnSpecGenerator implements SpawnSpecGenerator {

        /**
         * The vehicle specification
         */
        private VehicleSpec vehicleSpec;
        /**
         * The destination selector
         */
        private DestinationSelector destinationSelector;
        /**
         * the probability of generating a vehicle in each spawn time step
         */
        private double prob;

        /**
         * Create a spawn spec generator that generates only one spec.
         *
         * @param vehicleSpecId the vehicle spec ID
         * @param trafficLevel the traffic level
         * @param destinationSelector the destination selector
         */
        public OneSpawnSpecGenerator(int vehicleSpecId,
                double trafficLevel,
                DestinationSelector destinationSelector) {
            vehicleSpec = VehicleSpecDatabase.getVehicleSpecById(vehicleSpecId);
            this.destinationSelector = destinationSelector;

            prob = trafficLevel * SimConfig.SPAWN_TIME_STEP;
            // Cannot generate more than one vehicle in each spawn time step
            assert prob <= 1.0;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public List<SpawnSpec> act(SpawnPoint spawnPoint, double timeStep) {
            List<SpawnSpec> result = new LinkedList<SpawnSpec>();

            double initTime = spawnPoint.getCurrentTime();
            for (double time = initTime; time < initTime + timeStep;
                    time += SimConfig.SPAWN_TIME_STEP) {
                if (Util.RANDOM_NUM_GEN.nextDouble() < prob) {
                    Road destinationRoad
                            = destinationSelector.selectDestination(spawnPoint.getLane());

                    result.add(new SpawnSpec(spawnPoint.getCurrentTime(),
                            vehicleSpec,
                            destinationRoad,
                            VEHICLE_TYPE.AUTO));
                }
            }

            return result;
        }

        @Override
        public void vehicleGenerated() {
            // TODO Auto-generated method stub

        }

        @Override
        public SpawnSpec act(SpawnPoint spawnPoint, double timeStep, VEHICLE_TYPE vehicleType, Road destinationRoad) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }

        @Override
        public List<SpawnSpec> act(SpawnPoint spawnPoint, double timeStep, VEHICLE_TYPE vehicleType) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }
    }

    /**
     * The spec generator that generates just one vehicle in the entire
     * simulation.
     */
    public static class OnlyOneSpawnSpecGenerator implements SpawnSpecGenerator {

        /**
         * The vehicle specification
         */
        private VehicleSpec vehicleSpec;
        /**
         * The destination road
         */
        private Road destinationRoad;
        /**
         * Whether the spec has been generated
         */
        private boolean isDone;

        /**
         * Create a spec generator that generates just one vehicle in the entire
         * simulation.
         *
         * @param vehicleSpecId the vehicle spec ID
         * @param destinationRoad the destination road
         */
        public OnlyOneSpawnSpecGenerator(int vehicleSpecId, Road destinationRoad) {
            vehicleSpec = VehicleSpecDatabase.getVehicleSpecById(vehicleSpecId);
            this.destinationRoad = destinationRoad;
            isDone = false;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public List<SpawnSpec> act(SpawnPoint spawnPoint, double timeStep) {
            List<SpawnSpec> result = new ArrayList<SpawnSpec>(1);
            if (!isDone) {
                isDone = true;
                result.add(new SpawnSpec(spawnPoint.getCurrentTime(),
                        vehicleSpec,
                        destinationRoad,
                        VEHICLE_TYPE.AUTO));
            }
            return result;
        }

        @Override
        public void vehicleGenerated() {
            // TODO Auto-generated method stub

        }

        @Override
        public SpawnSpec act(SpawnPoint spawnPoint, double timeStep, VEHICLE_TYPE vehicleType, Road destinationRoad) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }

        @Override
        public List<SpawnSpec> act(SpawnPoint spawnPoint, double timeStep, VEHICLE_TYPE vehicleType) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }
    }

    /**
     * The spawn spec generator that enumerates spawn spec.
     */
    public static class EnumerateSpawnSpecGenerator implements SpawnSpecGenerator {

        /**
         * The list of destination roads
         */
        private List<Road> destinationRoads;
        /**
         * The vehicle spec ID
         */
        int vehicleSpecId;
        /**
         * The destination road ID
         */
        int destinationRoadId;
        /**
         * The next spawn time
         */
        double nextSpawnTime;
        /**
         * The spawn period
         */
        double spawnPeriod;

        /**
         * Create a spawn spec generator that enumerates spawn spec.
         *
         * @param spawnPoint the spawn point
         * @param destinationRoads the list of destination roads
         * @param initSpawnTime the initial spawn time
         * @param spawnPeriod the spawn period
         */
        public EnumerateSpawnSpecGenerator(SpawnPoint spawnPoint,
                List<Road> destinationRoads,
                double initSpawnTime,
                double spawnPeriod) {
            this.destinationRoads = new ArrayList<Road>(destinationRoads.size());
            for (Road road : destinationRoads) {
                if (Debug.currentMap.getRoad(spawnPoint.getLane()).getDual() != road) {
                    this.destinationRoads.add(road);
                }
            }
            this.vehicleSpecId = 0;
            this.destinationRoadId = 0;
            this.nextSpawnTime = initSpawnTime;
            this.spawnPeriod = spawnPeriod;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public List<SpawnSpec> act(SpawnPoint spawnPoint, double timeStep) {
            List<SpawnSpec> result = new ArrayList<SpawnSpec>(1);
            if (spawnPoint.getCurrentTime() >= nextSpawnTime) {
                if (vehicleSpecId < VehicleSpecDatabase.getNumOfSpec()) {
                    VehicleSpec vehicleSpec
                            = VehicleSpecDatabase.getVehicleSpecById(vehicleSpecId);
                    Road destinationRoad
                            = destinationRoads.get(destinationRoadId);
                    result.add(new SpawnSpec(spawnPoint.getCurrentTime(),
                            vehicleSpec,
                            destinationRoad,
                            VEHICLE_TYPE.AUTO));
                    nextSpawnTime += spawnPeriod;
                    destinationRoadId++;
                    if (destinationRoadId >= destinationRoads.size()) {
                        destinationRoadId = 0;
                        vehicleSpecId++;
                    }
                }  // else don't spawn any vehicle
            } // else wait until next spawn time
            return result;
        }

        @Override
        public void vehicleGenerated() {
            // TODO Auto-generated method stub

        }

        @Override
        public SpawnSpec act(SpawnPoint spawnPoint, double timeStep, VEHICLE_TYPE vehicleType, Road destinationRoad) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }

        @Override
        public List<SpawnSpec> act(SpawnPoint spawnPoint, double timeStep, VEHICLE_TYPE vehicleType) {
            throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
        }
    }

    /////////////////////////////////
    // PUBLIC STATIC METHODS
    /////////////////////////////////
    public static boolean laneIntersect(int firstLaneIn, int firstLaneOut,
            int secondLaneIn, int secondLaneOut) {
        if (roadBoard == null) {
            // initialize this, if needed.
            roadBoard = new StandardIntersectionBoard();
        }

        return roadBoard.intersects(firstLaneIn, firstLaneOut, secondLaneIn, secondLaneOut);
    }

    public static void createAndUseDifferingLanesPerRoadBoard(Intersection inter) {
        roadBoard = new DifferingLanesPerRoadBoard(inter);
    }

    /**
     * Set the FCFS managers at all intersections.
     *
     * @param layout the map
     * @param currentTime the current time
     * @param config the reservation grid manager configuration
     * @param interArch object containing information about the architecture and
     * turning policies for an intersection
     */
    public static void setFCFSManagers(GridMap layout,
            double currentTime,
            ReservationGridManager.Config config, ArchIntersection interArch) {
        layout.removeAllManagers();
        Intersection inter = null;
        for (int column = 0; column < layout.getColumns(); column++) {
            for (int row = 0; row < layout.getRows(); row++) {
                List<Road> roads = layout.getRoads(column, row);
                RoadBasedIntersection intersection = new RoadBasedIntersection(roads);
                inter = intersection;
                RoadBasedTrackModel trajectoryModel
                        = new RoadBasedTrackModel(intersection);
                V2IManager im
                        = new V2IManager(intersection, trajectoryModel, currentTime,
                                config, layout.getImRegistry());
                intersection.registerIntersectionManager(im);
                for (Road road : im.getIntersection().getRoads()) {
                    for (Lane lane : road.getLanes()) {
                        setPolicyAndExitRestictionsForLane(lane, im, interArch);
                    }
                }
                im.setPolicy(new BasePolicy(im, new FCFSRequestHandler()));
                layout.setManager(column, row, im);
            }
        }
        if (layout.getColumns() == layout.getRows() && layout.getColumns() == 1 && inter != null) {
            createAndUseDifferingLanesPerRoadBoard(inter);
        }
    }

    /**
     * Set the bath managers at all intersections.
     *
     * @param layout the map
     * @param currentTime the current time
     * @param config the reservation grid manager configuration
     * @param processingInterval the processing interval
     * @param interArch object containing information about the architecture and
     * turning policies for an intersection
     */
    public static void setBatchManagers(GridMap layout,
            double currentTime,
            ReservationGridManager.Config config,
            double processingInterval, ArchIntersection interArch) {
        layout.removeAllManagers();
        Intersection inter = null;
        for (int column = 0; column < layout.getColumns(); column++) {
            for (int row = 0; row < layout.getRows(); row++) {
                List<Road> roads = layout.getRoads(column, row);
                RoadBasedIntersection intersection = new RoadBasedIntersection(roads);
                inter = intersection;
                RoadBasedTrackModel trajectoryModel
                        = new RoadBasedTrackModel(intersection);
                V2IManager im
                        = new V2IManager(intersection, trajectoryModel, currentTime,
                                config, layout.getImRegistry());
                intersection.registerIntersectionManager(im);
                for (Road road : im.getIntersection().getRoads()) {
                    for (Lane lane : road.getLanes()) {
                        setPolicyAndExitRestictionsForLane(lane, im, interArch);
                    }
                }
                RequestHandler rh
                        = new BatchModeRequestHandler(
                                new RoadBasedReordering(processingInterval),
                                new BatchModeRequestHandler.RequestStatCollector());
                im.setPolicy(new BasePolicy(im, rh));
                layout.setManager(column, row, im);
            }
        }
        if (layout.getColumns() == layout.getRows() && layout.getColumns() == 1 && inter != null) {
            createAndUseDifferingLanesPerRoadBoard(inter);
        }
    }

    /**
     * Set the approximate simple traffic light managers at all intersections.
     *
     * @param layout the map
     * @param currentTime the current time
     * @param config the reservation grid manager configuration
     * @param greenLightDuration the green light duration
     * @param yellowLightDuration the yellow light duration
     * @param interArch object containing information about the architecture and
     * turning policies for an intersection
     */
    public static void setApproxSimpleTrafficLightManagers(
            GridMap layout,
            double currentTime,
            ReservationGridManager.Config config,
            double greenLightDuration,
            double yellowLightDuration, ArchIntersection interArch) {

        layout.removeAllManagers();
        Intersection inter = null;
        for (int column = 0; column < layout.getColumns(); column++) {
            for (int row = 0; row < layout.getRows(); row++) {
                List<Road> roads = layout.getRoads(column, row);
                RoadBasedIntersection intersection = new RoadBasedIntersection(roads);
                inter = intersection;
                RoadBasedTrackModel trajectoryModel
                        = new RoadBasedTrackModel(intersection);
                V2IManager im
                        = new V2IManager(intersection, trajectoryModel, currentTime,
                                config, layout.getImRegistry());
                intersection.registerIntersectionManager(im);
                for (Road road : im.getIntersection().getRoads()) {
                    for (Lane lane : road.getLanes()) {
                        setPolicyAndExitRestictionsForLane(lane, im, interArch);
                    }
                }
                ApproxSimpleTrafficSignalRequestHandler requestHandler
                        = new ApproxSimpleTrafficSignalRequestHandler(greenLightDuration,
                                yellowLightDuration);
                im.setPolicy(new BasePolicy(im, requestHandler));
                layout.setManager(column, row, im);
            }
        }
        if (layout.getColumns() == layout.getRows() && layout.getColumns() == 1 && inter != null) {
            createAndUseDifferingLanesPerRoadBoard(inter);
        }
    }

    /**
     * Set the approximate 4 phases traffic light managers at all intersections.
     *
     * @param layout the map
     * @param currentTime the current time
     * @param config the reservation grid manager configuration
     * @param greenLightDuration the green light duration
     * @param yellowLightDuration the yellow light duration
     * @param interArch object containing information about the architecture and
     * turning policies for an intersection
     */
    public static void setApprox4PhasesTrafficLightManagers(
            GridMap layout,
            double currentTime,
            ReservationGridManager.Config config,
            double greenLightDuration,
            double yellowLightDuration, ArchIntersection interArch) {
        layout.removeAllManagers();
        Intersection inter = null;
        for (int column = 0; column < layout.getColumns(); column++) {
            for (int row = 0; row < layout.getRows(); row++) {
                List<Road> roads = layout.getRoads(column, row);
                RoadBasedIntersection intersection = new RoadBasedIntersection(roads);
                inter = intersection;
                RoadBasedTrackModel trajectoryModel
                        = new RoadBasedTrackModel(intersection);
                V2IManager im
                        = new V2IManager(intersection, trajectoryModel, currentTime,
                                config, layout.getImRegistry());
                intersection.registerIntersectionManager(im);
                for (Road road : im.getIntersection().getRoads()) {
                    for (Lane lane : road.getLanes()) {
                        setPolicyAndExitRestictionsForLane(lane, im, interArch);
                    }
                }
                Approx4PhasesTrafficSignalRequestHandler requestHandler
                        = new Approx4PhasesTrafficSignalRequestHandler(greenLightDuration,
                                yellowLightDuration);
                im.setPolicy(new BasePolicy(im, requestHandler));
                layout.setManager(column, row, im);
            }
        }
        if (layout.getColumns() == layout.getRows() && layout.getColumns() == 1 && inter != null) {
            createAndUseDifferingLanesPerRoadBoard(inter);
        }
    }

    /**
     * Set the approximate N phases traffic light managers at all intersections.
     *
     * @param layout the map
     * @param currentTime the current time
     * @param config the reservation grid manager configuration
     * @param trafficSignalPhaseFileName the name of the file contains the
     * traffic signals duration information
     * @param interArch object containing information about the architecture and
     * turning policies for an intersection
     */
    public static void setApproxNPhasesTrafficLightManagers(
            GridMap layout,
            double currentTime,
            ReservationGridManager.Config config,
            String trafficSignalPhaseFileName, ArchIntersection interArch) {

        layout.removeAllManagers();
        Intersection inter = null;
        RingAndBarrier ringAndBarrier = null;
        if (layout.getColumns() > 0 && layout.getRows() > 0) {
            ringAndBarrier = RingAndBarrierFactory.createRingAndBarrierFromFile(layout, trafficSignalPhaseFileName);
            Resources.ringAndBarrier = ringAndBarrier;
        }

        for (int column = 0; column < layout.getColumns(); column++) {
            for (int row = 0; row < layout.getRows(); row++) {
                List<Road> roads = layout.getRoads(column, row);
                RoadBasedIntersection intersection = new RoadBasedIntersection(roads);
                inter = intersection;
                RoadBasedTrackModel trajectoryModel
                        = new RoadBasedTrackModel(intersection);
                V2IManager im
                        = new V2IManager(intersection, trajectoryModel, currentTime,
                                config, layout.getImRegistry(), ringAndBarrier);
                intersection.registerIntersectionManager(im);
                ApproxNPhasesTrafficSignalRequestHandler requestHandler;
                if (interArch == null) {
                    requestHandler = new ApproxNPhasesTrafficSignalRequestHandler(im);
                } else {
                    requestHandler = new ApproxNPhasesTrafficSignalRequestHandler(interArch.getMaxNumberOfLanes(), im);
                }

                for (Road road : im.getIntersection().getEntryRoads()) {
                    for (Lane lane : road.getLanes()) {
                        if (SimConfig.signalType == SimConfig.SIGNAL_TYPE.FULLY_ACTUATED) {
                            FullyActuatedSignalController controller
                                    = new FullyActuatedSignalController(ringAndBarrier, lane, im.getIntersection().getIntersectionManager(), requestHandler);
                            requestHandler.setSignalControllers(lane.getId(), controller);
                        } else if (SimConfig.signalType == SimConfig.SIGNAL_TYPE.RED_PHASE_ADAPTIVE) {
                            RedPhaseData.readRedPhaseData();
                            ringAndBarrier.LEGACY_resetRedDurations(RedPhaseData.defaultRedPhaseTime);
                            requestHandler.setSignalControllers(lane.getId(), ringAndBarrier.LEGACY_calcCyclicSignalController(road));
                        } else if (SimConfig.signalType == SimConfig.SIGNAL_TYPE.ONE_LANE_VERSION) {
                            OneLaneSignalController controller
                                    = new OneLaneSignalController(lane.getId(), OneLaneTimeConfig.greenTime, OneLaneTimeConfig.redTime);
                            requestHandler.setSignalControllers(lane.getId(), controller);
                        } else if (SimConfig.signalType == SimConfig.SIGNAL_TYPE.REVISED_PHASE) {
                            RevisedPhaseSignalController controller
                                    = RevisedPhaseConfig.getController(lane.getId());
                            requestHandler.setSignalControllers(lane.getId(), controller);
                        } else if (SimConfig.signalType == SimConfig.SIGNAL_TYPE.TRADITIONAL) {
                            ringAndBarrier.LEGACY_resetRedDurations(SimConfig.RED_PHASE_LENGTH);
                            requestHandler.setSignalControllers(lane.getId(), ringAndBarrier.LEGACY_calcCyclicSignalController(road));
                        } else if (SimConfig.signalType == SIGNAL_TYPE.HUMAN_ADAPTIVE) {
                            SignalController controller = AdaptiveTrafficSignalSuperviser.addTrafficSignalController(lane);
                            requestHandler.setSignalControllers(lane.getId(), controller);
                        } else if (SimConfig.DEDICATED_LANES > 0) {
                            DedicatedLanesSignalController controller
                                    = new DedicatedLanesSignalController(lane.getId());
                            requestHandler.setSignalControllers(lane.getId(), controller);
                        } else {
                            requestHandler.setSignalControllers(lane.getId(), ringAndBarrier.LEGACY_calcCyclicSignalController(road));
                        }
                        setPolicyAndExitRestictionsForLane(lane, im, interArch);
                    }
                }

                im.setPolicy(new BasePolicy(im, requestHandler));
                layout.setManager(column, row, im);
                Resources.im = im;
                if (SimConfig.signalType == SimConfig.SIGNAL_TYPE.FULLY_ACTUATED || Resources.ringAndBarrier != null) {
                    if(!ringAndBarrier.bindPhaseSegmentsToIM()) {
                        throw new RuntimeException("Couldn't bind IM to segments from the ring and barrier object.");
                    }
                }
            }
        }
        if (layout.getColumns() == layout.getRows() && layout.getColumns() == 1 && inter != null) {
            createAndUseDifferingLanesPerRoadBoard(inter);
        }
    }

    /**
     * Set the approximate N phases traffic light managers at all intersections.
     *
     * @param layout the map
     * @param currentTime the current time
     * @param config the reservation grid manager configuration
     * @param interArch object containing information about the architecture and
     * turning policies for an intersection
     */
    public static void setApproxStopSignManagers(GridMap layout,
            double currentTime,
            ReservationGridManager.Config config, ArchIntersection interArch) {
        layout.removeAllManagers();
        Intersection inter = null;
        for (int column = 0; column < layout.getColumns(); column++) {
            for (int row = 0; row < layout.getRows(); row++) {
                List<Road> roads = layout.getRoads(column, row);
                RoadBasedIntersection intersection = new RoadBasedIntersection(roads);
                inter = intersection;
                RoadBasedTrackModel trajectoryModel
                        = new RoadBasedTrackModel(intersection);
                V2IManager im
                        = new V2IManager(intersection, trajectoryModel, currentTime,
                                config, layout.getImRegistry());
                intersection.registerIntersectionManager(im);
                for (Road road : im.getIntersection().getRoads()) {
                    for (Lane lane : road.getLanes()) {
                        setPolicyAndExitRestictionsForLane(lane, im, interArch);
                    }
                }
                ApproxStopSignRequestHandler requestHandler
                        = new ApproxStopSignRequestHandler();
                im.setPolicy(new BasePolicy(im, requestHandler));
                layout.setManager(column, row, im);
            }
        }
        if (layout.getColumns() == layout.getRows() && layout.getColumns() == 1 && inter != null) {
            createAndUseDifferingLanesPerRoadBoard(inter);
        }
    }

    public static void setPolicyAndExitRestictionsForLane(Lane lane, V2IManager im, ArchIntersection interArch) {
        if (interArch != null && lane != null && im != null) {
            if (!interArch.isSetup()) {
                interArch.tieIntersection(im.getIntersection());
            }
            LaneIM laIM = lane.getLaneIM(); //lane intersection manager relationship, NOT the im.
            Iterable<TurnDirection> mappingKeys = interArch.getExitMappingKeys(lane);
            if (mappingKeys != null) {
                for (TurnDirection key : mappingKeys) {
                    laIM.setExitLaneMapping(im, key, interArch.getExitLane(lane, key));
                }
            } else {
                //if lane mappings weren't specified, set no outgoing lane allowed for the mapping
                laIM.disallowAllOutgoingLaneMappings(im);
            }

            Iterable<VEHICLE_TYPE> vTypeKeys = interArch.getTurnPolicyKeys(lane);
            if (vTypeKeys != null) {
                for (VEHICLE_TYPE vType : vTypeKeys) {
                    laIM.setValidActionsForLane(im, interArch.getAllowedTurns(lane, vType), vType);
                }
            } else {
                //if actions weren't specified, set all actions disallowed for the lane
                for (VEHICLE_TYPE vType : VEHICLE_TYPE.values()) {
                    laIM.setValidActionsForLane(im, null, vType);
                }
            }
        }
    }

    /**
     * Set the uniform random spawn points.
     *
     * @param map the map
     * @param trafficLevel the traffic level
     */
    public static void setUniformRandomSpawnPoints(GridMap map,
            double trafficLevel) {
        for (SpawnPoint sp : map.getSpawnPoints()) {
            sp.setVehicleSpecChooser(
                    new UniformSpawnSpecGenerator(trafficLevel,
                            new RandomDestinationSelector(map)));
        }
    }

    /**
     * Set Up File Destination spawn points
     *
     * @param map the map
     * @param tms object encapsulating turn movements to use for spawning with
     * the FileBasedSpawnSpecGenerator
     */
    public static void setSpawnDestSpawnPoints(BasicMap map,
            TurnMovements tms) {

        DestinationFileSpawnSpecGenerator destspecgen = TrafficFlowReaderFactory.getDestFileSpawnSpecGen(tms.getCountLists(), new FileBasedDestinationSelector(map.getImRegistry()));

        for (SpawnPoint sp : map.getSpawnPoints()) {
            sp.setVehicleSpecChooser(destspecgen);
        }
    }

    /**
     * Set Up Lane Restricted File Destination spawn points
     *
     * @param map the map
     * @param tms object encapsulating turn movements to use for spawning with
     * the FileBasedSpawnSpecGenerator
     */
    public static void setLaneRestrictedSpawnDestSpawnPoints(BasicMap map,
            TurnMovements tms) {

        LaneRestrictedFileSpawnSpecGenerator destspecgen = TrafficFlowReaderFactory.getLaneRestrDestFileSpawnSpecGen(tms.getCountLists(), new FileBasedDestinationSelector(map.getImRegistry()));

        for (SpawnPoint sp : map.getSpawnPoints()) {
            sp.setVehicleSpecChooser(destspecgen);
        }
    }

    /**
     * Set the uniform turn based spawn points.
     *
     * @param map the map
     * @param trafficLevel the traffic level
     */
    public static void setUniformTurnBasedSpawnPoints(GridMap map,
            double trafficLevel) {
        for (SpawnPoint sp : map.getSpawnPoints()) {
            sp.setVehicleSpecChooser(
                    new UniformSpawnSpecGenerator(trafficLevel,
                            new TurnBasedDestinationSelector(map)));
        }
    }

    /**
     * Set the uniform ratio spawn points with various traffic volume.
     *
     * @param map the map
     * @param trafficVolumeFileName the traffic volume filename
     * @param trafficLevel	the traffic level
     */
    public static void setUniformRatioSpawnPoints(GridMap map,
            String trafficVolumeFileName,
            double trafficLevel) {
        // no worry - (traffic volume / total traffic volume) is the useful information
        // so, how much a specific traffic volume is does not make any sense
        TrafficVolume trafficVolume = TrafficVolume.makeVolume(map, trafficVolumeFileName);

        //DestinationSelector selector = new RatioDestinationSelector(map,
        //trafficVolume);
        DestinationSelector selector = new TurnBasedDestinationSelector(map);

        // see how much volume in total - this helps to figure out portion of each sp
        double totalVolume = 0;
        int spNum = map.getSpawnPoints().size();

        LaneTrafficController trafficController;

        // check the lane spawning setting - dedicated lanes or not
        if (SimConfig.DEDICATED_LANES > 0) {
            trafficController = new DedicatedTrafficController(
                    trafficLevel, SimConfig.HUMAN_PERCENTAGE, SimConfig.CONSTANT_HUMAN_PERCENTAGE);
        } else {
            trafficController = new NormalTrafficController(
                    trafficLevel,
                    SimConfig.HUMAN_PERCENTAGE,
                    SimConfig.CONSTANT_HUMAN_PERCENTAGE,
                    SimConfig.ADAPTIVE_HUMAN_PERCENTAGE,
                    trafficVolume);
        }

        // find out traffic level for each spawning point
        for (SpawnPoint sp : map.getSpawnPoints()) {

            int laneId = sp.getLane().getId();

            sp.setVehicleSpecChooser(
                    new UniformSpawnSpecGenerator(trafficController.getLaneInfo(laneId), trafficLevel, selector));
        }

        currentTrafficLevel = trafficLevel;
    }

    /**
     * Set the directional spawn points which has different traffic volumes in
     * different directions.
     *
     * @param layout the map
     * @param hTrafficLevel the traffic level in the horizontal direction
     * @param vTrafficLevel the traffic level in the vertical direction
     */
    public static void setDirectionalSpawnPoints(GridMap layout,
            double hTrafficLevel,
            double vTrafficLevel) {
        for (SpawnPoint sp : layout.getHorizontalSpawnPoints()) {
            sp.setVehicleSpecChooser(
                    new UniformSpawnSpecGenerator(hTrafficLevel,
                            new RandomDestinationSelector(layout)));
        }
        for (SpawnPoint sp : layout.getVerticalSpawnPoints()) {
            sp.setVehicleSpecChooser(
                    new UniformSpawnSpecGenerator(vTrafficLevel,
                            new RandomDestinationSelector(layout)));
        }
    }

    /**
     * Set the baseline spawn points.
     *
     * @param layout the map
     * @param traversalTime the traversal time
     */
    public static void setBaselineSpawnPoints(GridMap layout,
            double traversalTime) {
        int totalNumOfLanes = 0;
        int minNumOfLanes = Integer.MAX_VALUE;
        for (Road r : layout.getRoads()) {
            int n = r.getLanes().size();
            totalNumOfLanes += n;
            if (n < minNumOfLanes) {
                minNumOfLanes = n;
            }
        }
        double numOfTraversals
                = VehicleSpecDatabase.getNumOfSpec() * (totalNumOfLanes - minNumOfLanes);

        for (SpawnPoint sp : layout.getSpawnPoints()) {
            sp.setVehicleSpecChooser(
                    new EnumerateSpawnSpecGenerator(
                            sp,
                            layout.getDestinationRoads(),
                            sp.getLane().getId() * traversalTime * numOfTraversals,
                            traversalTime));
        }
    }

    public static double getTrafficLevel() {
        return currentTrafficLevel;
    }
}
