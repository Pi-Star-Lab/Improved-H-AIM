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
package aim4.im.v2i.RequestHandler;

import aim4.config.Constants;
import aim4.config.Constants.TurnDirection;
import aim4.config.Debug;
import aim4.config.Resources;
import aim4.config.SimConfig;
import aim4.config.TrafficSignal;
import aim4.config.SimConfig.SIGNAL_TYPE;
import aim4.config.SimConfig.VEHICLE_TYPE;
import aim4.config.ringbarrier.RBSegmentReadOnlyNoLockingView;
import aim4.driver.Driver;
import aim4.driver.coordinator.V2ICoordinator.State;
import aim4.im.IntersectionManager;

import java.util.List;

import aim4.im.v2i.policy.BasePolicy;
import aim4.im.v2i.policy.BasePolicyCallback;
import aim4.im.v2i.policy.BasePolicy.ProposalFilterResult;
import aim4.im.v2i.policy.BasePolicy.ReserveParam;
import aim4.map.GridMapUtil;
import aim4.map.Road;
import aim4.map.lane.Lane;
import aim4.msg.i2v.Reject;
import aim4.msg.v2i.Request;
import aim4.sim.StatCollector;
import aim4.util.Registry;
import aim4.util.Util;
import aim4.vehicle.VehicleSimView;
import expr.trb.DesignatedLanesExpr;
import expr.trb.TrafficSignalExpr;
import java.awt.geom.Point2D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.Set;

/**
 * The approximate N-Phases traffic signal request handler.
 */
public class ApproxNPhasesTrafficSignalRequestHandler implements
        TrafficSignalRequestHandler {

    /////////////////////////////////
    // NESTED CLASSES
    /////////////////////////////////
    /**
     * The interface of signal controllers.
     */
    public static interface SignalController {

        /**
         * Get the signal at the given time
         *
         * @param time the given time
         * @return the signal
         */
        TrafficSignal getSignal(double time);

        /**
         * Get the permissiveness of signal in terms of cross turning (left in
         * US) at a specific time
         *
         * @param time the given time
         * @return true if cross turns with oncoming traffic are allowed
         */
        boolean getSignalPermissiveness(double time);

        /**
         * Set the offset
         *
         * @param time
         */
        void setOffset(double time);
    }

    /**
     * The cyclic signal controller.
     */
    public static class CyclicSignalController implements SignalController {

        /**
         * The durations of the signals
         */
        private double[] durations;
        /**
         * The list of signals
         */
        private TrafficSignal[] signals;
        /**
         * The list of permissive cross (left in US) turning phases
         */
        private boolean[] permissive;
        /**
         * The duration offset
         */
        private static double durationOffset;
        /**
         * The total duration
         */
        private static double totalDuration;

        public CyclicSignalController(double[] durations, TrafficSignal[] signals, boolean[] permissive) {
            this(durations, signals, 0.0, permissive);
        }

        public CyclicSignalController(double[] durations, TrafficSignal[] signals) {
            this(durations, signals, 0.0, null);
        }

        /**
         * Create a cyclic signal controller.
         *
         * @param durations the durations of the signals
         * @param signals the list of signals
         * @param durationOffset the duration offset
         * @param permissive the cross (left in US) turn permissiveness' of
         * signal
         */
        public CyclicSignalController(double[] durations, TrafficSignal[] signals,
                double durationOffset, boolean[] permissive) {
            this.durations = durations.clone();
            this.signals = signals.clone();
            this.durationOffset = durationOffset;
            if (permissive == null) {
                this.permissive = new boolean[durations.length]; //should be the same length as both traffic signal and durations
                Arrays.fill(this.permissive, false);
            } else {
                this.permissive = permissive.clone();
            }

            totalDuration = 0.0;
            for (double d : durations) {
                totalDuration += d;
            }
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public TrafficSignal getSignal(double time) {
            time -= durationOffset;
            double d = time % totalDuration;
            assert 0.0 <= d && d < totalDuration;
            double maxd = 0.0;
            for (int i = 0; i < durations.length; i++) {
                maxd += durations[i];
                if (d < maxd) {
                    return signals[i];
                }
            }
            assert false : ("Error in CyclicLightController()");
            return null;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean getSignalPermissiveness(double time) {
            time -= durationOffset;
            double d = time % totalDuration;
            assert 0.0 <= d && d < totalDuration;
            double maxd = 0.0;
            for (int i = 0; i < durations.length; i++) {
                maxd += durations[i];
                if (d < maxd) {
                    return permissive[i];
                }
            }
            assert false : ("Error in CyclicLightController()");
            return false;
        }

        /**
         * Return whether the current time exceeds this total duration. If so,
         * it needs to re-calculate the red phase time.
         *
         * @param currentTime the current time
         * @return
         */
        public static boolean needRecalculate(double currentTime) {
            if (currentTime > durationOffset + totalDuration) {
                return true;
            } else {
                return false;
            }
        }

        /**
         * A new round would calculate from this end time - this is the offset
         * of next round
         *
         * @return the time of end of this round
         */
        public static double getEndTime() {
            return durationOffset + totalDuration;
        }

        @Override
        public void setOffset(double time) {
            durationOffset = time;
        }
    }

    /**
     * The class that green signal is on one lane by one. This is leaving enough
     * space for autonomous vehicles to pass thorugh the intersection because of
     * enough space left for them.
     *
     * @author menie
     *
     */
    public static class OneLaneSignalController extends AbstractSignalControllerWithLaneID {

        /**
         * information necessary for this controller
         */
        private double greenTime;
        private double redTime;
        private double totalTime;
        private int laneNum;

        public OneLaneSignalController(int id, double greenTime, double redTime) {
            super(id);
            this.greenTime = greenTime;
            this.redTime = redTime;
            this.totalTime = greenTime + redTime;

            this.laneNum = Resources.map.getLaneRegistry().getValues().size();
        }

        @Override
        public TrafficSignal getSignal(double time) {
            if (Math.ceil(time / totalTime) % laneNum == laneId) {
                if (time % this.totalTime < greenTime) {
                    return TrafficSignal.GREEN;
                } else {
                    return TrafficSignal.RED;
                }
            } else {
                return TrafficSignal.RED;
            }
        }

        @Override
        public void setOffset(double time) {
            // TODO Auto-generated method stub

        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean getSignalPermissiveness(double time) {
            return false;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public int getLaneID() {
            return laneId;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public Lane getLane() {
            return Resources.map.getLaneRegistry().get(laneId);
        }

    }

    /**
     * For revised phase signal policy. 1. two opposing directions without
     * turning left ones 2. turning left ones
     *
     * @author menie
     *
     */
    public static class RevisedPhaseSignalController implements SignalController {

        /**
         * Some useful variables
         */
        private double greenDuration[];
        private double totalTime;

        public RevisedPhaseSignalController(double[] greenDuration, double totalTime) {
            this.greenDuration = greenDuration;
            this.totalTime = totalTime;
        }

        @Override
        public TrafficSignal getSignal(double time) {
            double localTime = time % totalTime;

            if (localTime >= greenDuration[0] && localTime < greenDuration[1]) {
                return TrafficSignal.GREEN;
            } else {
                return TrafficSignal.RED;
            }
        }

        @Override
        public void setOffset(double time) {
            // TODO Auto-generated method stub

        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean getSignalPermissiveness(double time) {
            return false;
        }

    }

    public static class AdaptiveSignalController implements SignalController {

        private ArrayList<ArrayList<Double>> greenPhaseDuration = new ArrayList<ArrayList<Double>>();

        @Override
        public TrafficSignal getSignal(double time) {
            for (ArrayList<Double> duration : greenPhaseDuration) {
                if (duration.get(0) < time && time <= duration.get(1)) {
                    return TrafficSignal.GREEN;
                }
            }

            // otherwise, it's not a green phase
            return TrafficSignal.RED;
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean getSignalPermissiveness(double time) {
            return false;
        }

        public void prepareGreenPhase(double start, double end) {
            ArrayList<Double> duration = new ArrayList<Double>();
            duration.add(start);
            duration.add(end);

            greenPhaseDuration.add(duration);
        }

        @Override
        public void setOffset(double time) {
            // TODO Auto-generated method stub

        }

    }

    public static class DedicatedLanesSignalController implements SignalController {

        private double greenTime = 15;
        private double redIntervalTime = 2;
        private double redTime = 15;
        private double totalTime;
        private int rank; // No. ? to turn on green light
        private boolean forHuman;

        public DedicatedLanesSignalController(int lane) {
            int laneNum = Resources.map.getLaneRegistry().getValues().size();
            rank = lane / (laneNum / 4);
            forHuman = (lane % (laneNum / 4)) < SimConfig.DEDICATED_LANES;

            totalTime = greenTime * 8 + redIntervalTime * 3 + redTime;
        }

        @Override
        public TrafficSignal getSignal(double time) {
            double timeInPeriod = time % totalTime;

            if (SimConfig.DEDICATED_LANES > 0) {
                // check whether need to change signal time
                if (timeInPeriod < greenTime * 8 + redIntervalTime * 3) {
                    SimConfig.signalType = SIGNAL_TYPE.DEFAULT;
                } else {
                    SimConfig.signalType = SIGNAL_TYPE.TRADITIONAL;
                }
            }

            // after 4 green phases, or it's not a human lane, return red phase
            if (timeInPeriod > greenTime * 8 + redIntervalTime * 3) {
                return TrafficSignal.RED;
            } else {
                int round = (int) (timeInPeriod / (greenTime * 2 + redIntervalTime));
                double timeInRound = timeInPeriod % (greenTime * 2 + redIntervalTime);

                if (round == rank) {
                    if (timeInRound < greenTime && forHuman) {
                        return TrafficSignal.GREEN;
                    } else if (timeInRound > greenTime && timeInRound < greenTime * 2 && !forHuman) {
                        return TrafficSignal.GREEN;
                    } else {
                        return TrafficSignal.RED;
                    }
                } else {
                    return TrafficSignal.RED;
                }
            }
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public boolean getSignalPermissiveness(double time) {
            return false;
        }

        @Override
        public void setOffset(double time) {
            // TODO Auto-generated method stub

        }
    }
    /////////////////////////////////
    // PRIVATE FIELDS
    /////////////////////////////////

    /**
     * A mapping from lane ID to the traffic signal controllers on the lane.
     */
    private Map<Integer, SignalController> signalControllers;
    /**
     * The base policy
     */
    private BasePolicyCallback basePolicy;
    /**
     * The maximum number of lanes across any road at the intersection
     */
    private int numberOfLanes;

    private IntersectionManager im;

    /////////////////////////////////
    // CONSTRUCTORS
    /////////////////////////////////
    /**
     * Create the approximate N-Phases traffic signal request handler.
     *
     * @param maxNumLanes The maximum number of lanes across any road at the
     * intersection
     * @param im
     */
    public ApproxNPhasesTrafficSignalRequestHandler(int maxNumLanes, IntersectionManager im) {
        signalControllers = new HashMap<Integer, SignalController>();
        Resources.signalControllers = signalControllers;
        numberOfLanes = maxNumLanes;
        this.im = im;
    }

    /**
     * Create the approximate N-Phases traffic signal request handler, uses
     * DesignatedLanesExpr.NUMBER_OF_LANES as max number of lanes.
     *
     * @param im
     */
    public ApproxNPhasesTrafficSignalRequestHandler(IntersectionManager im) {
        this(DesignatedLanesExpr.NUMBER_OF_LANES, im);
    }

    /////////////////////////////////
    // PUBLIC METHODS
    /////////////////////////////////
    /**
     * {@inheritDoc}
     */
    @Override
    public void act(double timeStep) {
        // do nothing
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setBasePolicyCallback(BasePolicyCallback basePolicy) {
        this.basePolicy = basePolicy;
    }

    /**
     * Set the traffic signal controller of a lane
     *
     * @param laneId the lane ID
     * @param signalController the signal controller
     */
    public void setSignalControllers(int laneId,
            SignalController signalController) {
        signalControllers.put(laneId, signalController);
    }

    public double getCurrentTime() {
        return basePolicy.getCurrentTime();
    }

    /*Only handles AVs and HVs without comm drop probability, is V2*/
    /**
     * {@inheritDoc}
     */
    @Override
    public void processRequestMsg(Request msg) {
        int vin = msg.getVin();
        boolean avEnteringPrecariously = false;
        // If the vehicle has got a reservation already, reject it.
        if (basePolicy.hasReservation(vin)) {
            basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                    Reject.Reason.CONFIRMED_ANOTHER_REQUEST);
            return;
        }

        // filter the proposals
        ProposalFilterResult filterResult
                = BasePolicy.generousFilter(msg.getProposals(),
                        basePolicy.getCurrentTime());
        if (filterResult.isNoProposalLeft()) {
            basePolicy.sendRejectMsg(vin,
                    msg.getRequestId(),
                    filterResult.getReason());
            return;
        }

        List<Request.Proposal> proposals = filterResult.getProposals();

        // double check
        if (proposals == null) {
            return;
        }
        if (!canEnterFromLane(proposals.get(0).getArrivalLaneID(),
                proposals.get(0).getDepartureLaneID(), Resources.vinToVehicles.get(vin).getVehicleType(), null, false)
                && proposals.get(0).getArrivalTime()
                > basePolicy.getCurrentTime()
                + im.getMaxAllowedFutureReservationTimeOnLane(proposals.get(0).getArrivalLane())) {
            basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                    Reject.Reason.NO_CLEAR_PATH);
            return;
        }

        if (// we need to see if the vehicle is allowed to enter the intersection at a specific time
                !canEnterFromLaneAtTimepoint(proposals.get(0).getArrivalLaneID(),
                        proposals.get(0).getDepartureLaneID(),
                        proposals.get(0).getArrivalTime(),
                        proposals.get(0).getIntersectionManager(), Resources.vinToVehicles.get(vin).getVehicleType(), false)) {

            // it's now red light. it's human and not turning right => wait here!
            if (Resources.vinToVehicles.get(vin).isHuman() && (!SimConfig.ALLOW_RIGHT_TURNS_ON_RED_FOR_ANY_VEHICLE_TYPE || !makingRightTurn(proposals.get(0).getArrivalLaneID(),
                    proposals.get(0).getDepartureLaneID()))) {
                basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                        Reject.Reason.NO_CLEAR_PATH);
                return;
            }
            avEnteringPrecariously = true;
        }

        // try to see if reservation is possible for the remaining proposals.
        ReserveParam reserveParam = basePolicy.findReserveParam(msg, proposals);

        if (reserveParam != null) {

            double exitTime = reserveParam.getExitTime();
            boolean isMakingRightTurn = makingRightTurn(reserveParam.getSuccessfulProposal().getArrivalLaneID(), reserveParam.getSuccessfulProposal().getDepartureLaneID());
            if (!SimConfig.ALLOW_RIGHT_TURNS_ON_RED_FOR_ANY_VEHICLE_TYPE && avEnteringPrecariously && isMakingRightTurn) {
                Lane arrivalLane = DesignatedLanesExpr.laneRegistry.get(proposals.get(0).getArrivalLaneID());
                //check if the humans have a green light, if they do proceed on below where we'll check if we might be hindering them.
                TrafficSignal ts = ((FullyActuatedSignalController) signalControllers.get(arrivalLane.getId())).getSignal(proposals.get(0).getArrivalTime(), EnumSet.of(Constants.TurnDirection.RIGHT));
                if (ts != TrafficSignal.GREEN) {
                    basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                            Reject.Reason.NO_CLEAR_PATH);
                    return;
                }
            }

            //handle edge case where comm range is near the whole visible map, auto vehicle spawns on road A with speed limit <= road B, makes a reservation and succeeds immediately, and then a human vehicle spawns on road B and will meet or beat the auto vehicle to the intersection due to difference in speed limits
            //this is: time to exit the intersection msut be < the time an unseen vehicle could spawn in and make it to the intersection
            if (exitTime >= basePolicy.getCurrentTime() + im.getMaxAllowedFutureReservationTimeOnLane(reserveParam.getSuccessfulProposal().getArrivalLane())) {
                basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                        Reject.Reason.ARRIVAL_TIME_DOESNT_ACCOUNT_FOR_UNSPAWNED_VEHICLES);
                return;
            } else if (shouldYieldForPermissiveCrossTurn(proposals.get(0).getArrivalLaneID(), proposals.get(0).getDepartureLaneID(), proposals.get(0).getIntersectionManager(), reserveParam.getSuccessfulProposal().getArrivalTime(), reserveParam.getExitTime(), Resources.vinToVehicles.get(vin))) {//|| shouldYieldToForRightTurnOnRed(proposals.get(0).getArrivalLaneID(), proposals.get(0).getDepartureLaneID(), proposals.get(0).getIntersectionManager(), reserveParam.getSuccessfulProposal().getArrivalTime(), reserveParam.getExitTime(), Resources.vinToVehicles.get(vin))) {
                basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                        Reject.Reason.MUST_YIELD_FOR_TURNING_ACTION);
                return;
            }

            boolean isMakingLeftTurn = makingLeftTurn(reserveParam.getSuccessfulProposal().getArrivalLaneID(), reserveParam.getSuccessfulProposal().getDepartureLaneID());
            //check if vehicle is AV and might hinder humans:
            //current time is used as the starting time for the range because AVs might arrive after an HV will have entered the intersection, but before the HV has left. Using current time makes it so we're looking from the current time all the way to the vehicle's exit to see if a human vehicle might enter
            if (avEnteringPrecariously && !notHinderingHumanVehiclesForTimeRange(reserveParam.getSuccessfulProposal().getArrivalLaneID(), reserveParam.getSuccessfulProposal().getDepartureLaneID(), basePolicy.getCurrentTime() /*reserveParam.getSuccessfulProposal().getArrivalTime())*/, exitTime, im, (avEnteringPrecariously && isMakingLeftTurn))) {
                basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                        Reject.Reason.NO_CLEAR_PATH);
                return;
            }

            //this is an inheritance sin, but, ¯\_(ツ)_/¯
            if (SimConfig.signalType == SIGNAL_TYPE.FULLY_ACTUATED) {
                FullyActuatedSignalController fasc = ((FullyActuatedSignalController) signalControllers.get(reserveParam.getSuccessfulProposal().getArrivalLaneID()));
                //fasc.logActuationOnLane(basePolicy.getCurrentTime(), reserveParam.getSuccessfulProposal().getArrivalTime(), reserveParam.getExitTime(), Resources.vinToVehicles.get(vin).getVehicleType());
            }
            basePolicy.sendComfirmMsg(msg.getRequestId(), reserveParam);
            DesignatedLanesExpr.laneRegistry.get(proposals.get(0).getArrivalLaneID()).exit(0);
            DesignatedLanesExpr.laneRegistry.get(proposals.get(0).getDepartureLaneID()).enter(1);
        } else {
            basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                    Reject.Reason.NO_CLEAR_PATH);
        }
        //check if the AV is hindering human drivers
    }

    /**
     * {@inheritDoc}
     */
    public void processRequestMsg_Old(Request msg) {
        int vin = msg.getVin();
        boolean insertingTraffic = false; // for FCFS-SIGNAL when the vehicle is in red lane but get reservation

        // If the vehicle has got a reservation already, reject it.
        if (basePolicy.hasReservation(vin)) {
            basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                    Reject.Reason.CONFIRMED_ANOTHER_REQUEST);
            return;
        }

        // filter the proposals
        ProposalFilterResult filterResult
                = BasePolicy.generousFilter(msg.getProposals(),
                        basePolicy.getCurrentTime());
        if (filterResult.isNoProposalLeft()) {
            basePolicy.sendRejectMsg(vin,
                    msg.getRequestId(),
                    filterResult.getReason());
            return;
        }

        List<Request.Proposal> proposals = filterResult.getProposals();

        // double check
        if (proposals == null) {
            return;
        }

        if (!canEnterFromLane(proposals.get(0).getArrivalLaneID(),
                proposals.get(0).getDepartureLaneID(), Resources.vinToVehicles.get(vin).getVehicleType(), null, false)
                && proposals.get(0).getArrivalTime()
                > basePolicy.getCurrentTime()
                + im.getMaxAllowedFutureReservationTimeOnLane(proposals.get(0).getArrivalLane())) {
            basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                    Reject.Reason.NO_CLEAR_PATH);
            return;
        }
        if (TrafficSignalExpr.dropMessageProb > 0 && Util.RANDOM_NUM_GEN.nextDouble() < TrafficSignalExpr.dropMessageProb && !Resources.vinToVehicles.get(vin).isHuman()) {
            if (!canEnterFromLaneAtTimepoint(proposals.get(0).getArrivalLaneID(),
                    proposals.get(0).getDepartureLaneID(),
                    proposals.get(0).getArrivalTime(),
                    proposals.get(0).getIntersectionManager(), Resources.vinToVehicles.get(vin).getVehicleType(), false)) {

                basePolicy.sendRejectMsg(vin, msg.getRequestId(), Reject.Reason.DROPPED_MESSAGE);
                return;
            }
        } else if (SimConfig.signalType == SimConfig.SIGNAL_TYPE.DEFAULT) {
            // if this is SIGNAL and FCFS is not applied, check whether the light allows it to go across
            if (!canEnterFromLane(proposals.get(0).getArrivalLaneID(),
                    proposals.get(0).getDepartureLaneID(),
                    Resources.vinToVehicles.get(vin).getVehicleType(), null, false)) {
                // If cannot enter from lane according to canEnterFromLane(), reject it.
                basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                        Reject.Reason.NO_CLEAR_PATH);
                return;
            }
        } else {
            if (// to be frank, this vehicle cannot go through the intersection now,
                    // but we need to check more to confirm this, see the following part
                    !canEnterFromLaneAtTimepoint(proposals.get(0).getArrivalLaneID(),
                            proposals.get(0).getDepartureLaneID(),
                            proposals.get(0).getArrivalTime(),
                            proposals.get(0).getIntersectionManager(), Resources.vinToVehicles.get(vin).getVehicleType(), false)) {
                // it's now red light. it's human => wait here!
                if (Resources.vinToVehicles.get(vin).isHuman()
                        && !makingRightTurn(proposals.get(0).getArrivalLaneID(),
                                proposals.get(0).getDepartureLaneID())) {
                    basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                            Reject.Reason.NO_CLEAR_PATH);
                    return;
                }

                if (SimConfig.signalType == SimConfig.SIGNAL_TYPE.TRADITIONAL
                        || SimConfig.signalType == SimConfig.SIGNAL_TYPE.FULLY_ACTUATED
                        || SimConfig.signalType == SimConfig.SIGNAL_TYPE.ONE_LANE_VERSION
                        || SimConfig.signalType == SimConfig.SIGNAL_TYPE.REVISED_PHASE
                        || SimConfig.signalType == SimConfig.SIGNAL_TYPE.RED_PHASE_ADAPTIVE) {

                    if (SimConfig.FULLY_OBSERVING) {
                        // in this case, just check whether human drivers appear
//                        if (vin == 1006) {
//                            DesignatedLanesExpr.segmentIndex++;
//                            System.out.println(DesignatedLanesExpr.segmentIndex);
//                            if (DesignatedLanesExpr.segmentIndex == 222) {
//                                System.out.println(DesignatedLanesExpr.segmentIndex);
//
//                            }
//                        }
//                        if(vin == 1044){
//                            System.out.println(DesignatedLanesExpr.SEED++); 
//                            if(DesignatedLanesExpr.SEED == 2030){
//                                int i = -1;
//                            }
//                        }
//                        if (!notHinderingHumanVehicles(proposals.get(0).getArrivalLaneID(),
//                                proposals.get(0).getDepartureLaneID(),
//                                proposals.get(0).getArrivalTime())
//                                || !notHinderingCAV(proposals.get(0).getArrivalLaneID(),
//                                        proposals.get(0).getDepartureLaneID(),
//                                        proposals.get(0).getArrivalTime())) {
                        if (/*!canEnterFromLane(proposals.get(0).getArrivalLaneID(),
                                proposals.get(0).getDepartureLaneID(),
                                Resources.vinToVehicles.get(vin).getVehicleType(), null)
                                ||*/!notHinderingHumanVehicles(proposals.get(0).getArrivalLaneID(),
                                        proposals.get(0).getDepartureLaneID(),
                                        proposals.get(0).getArrivalTime(),
                                        proposals.get(0).getIntersectionManager())) {
                            basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                                    Reject.Reason.NO_CLEAR_PATH);
                            return;
                        }
                    } else {
                        // in this case, the intersection is not fully observable
                        // we need to assume there's always a human driven vehicle would appear.
                        if (!notHinderingPotentialHumanDrivers(proposals.get(0).getArrivalLaneID(),
                                proposals.get(0).getDepartureLaneID(),
                                proposals.get(0).getArrivalTime(),
                                proposals.get(0).getIntersectionManager(), Resources.vinToVehicles.get(vin).getVehicleType())) {
                            basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                                    Reject.Reason.NO_CLEAR_PATH);
                            return;
                        }
                    }
                }

                // for informed human vehicles in fully observing policy
                if (Resources.vinToVehicles.get(vin).withCruiseControll()) {

                    if ( // if he's turning left.. Don't do it!
                            makingLeftTurn(proposals.get(0).getArrivalLaneID(),
                                    proposals.get(0).getDepartureLaneID())
                            || // if he has been told to stop,
                            // he can no longer enter the intersection in the same red phase.
                            Resources.vinToVehicles.get(vin).hasStopped()) {

                        // reject
                        basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                                Reject.Reason.NO_CLEAR_PATH);
                        return;
                    }
                } else if (Resources.vinToVehicles.get(vin).withAdaptiveCruiseControll()) {

                    if ( // Make sure there is some vehicle in front of it for it to follow
                            !canFollowFrontVehicle(Resources.vinToVehicles.get(vin))
                            || // He cannot simply follow the vehicle in front of it in right lane,
                            // when the vehicle in front of it is a human-driven vehicle.
                            inRightLane(proposals.get(0).getArrivalLaneID(),
                                    proposals.get(0).getDepartureLaneID())
                            && Resources.vinToVehicles.get(vin).getFrontVehicle().isHuman()) {
                        // reject
                        basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                                Reject.Reason.NO_CLEAR_PATH);
                        return;
                    }
                }
                // wow, a brave action! it's going into the intersection in red light!
                // mark this, and we need to check if it's also safe when it exit
                insertingTraffic = true;
            }
        }

        // try to see if reservation is possible for the remaining proposals.
        ReserveParam reserveParam = basePolicy.findReserveParam(msg, proposals);
        // now, check whether everything is fine when it exit
        // it would be sorry if this vehicle enters the intersection, and in the middle of his road,
        // the traffic light on a intersecting road becomes green.
        if (reserveParam != null && (insertingTraffic || (Resources.vinToVehicles.get(vin).getVehicleType() == SimConfig.VEHICLE_TYPE.AUTO && getTurnDirection(proposals.get(0).getArrivalLaneID(), proposals.get(0).getDepartureLaneID()) == Constants.CROSS_TURN_DIRECTION))) {
            double exitTime = reserveParam.getExitTime();
            //handle edge case where comm range is near the whole visible map, auto vehicle spawns on road A with speed limit <= road B, makes a reservation and succeeds immediately, and then a human vehicle spawns on road B and will meet or beat the auto vehicle to the intersection due to difference in speed limits
            //this is: time to exit the intersection < the time an unseen vehicle could spawn in and make it to the intersection
            if (exitTime >= basePolicy.getCurrentTime() + im.getMaxAllowedFutureReservationTimeOnLane(reserveParam.getSuccessfulProposal().getArrivalLane())) {
                basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                        Reject.Reason.ARRIVAL_TIME_DOESNT_ACCOUNT_FOR_UNSPAWNED_VEHICLES);
                return;
            }

            if (SimConfig.signalType == SimConfig.SIGNAL_TYPE.TRADITIONAL
                    || SimConfig.signalType == SimConfig.SIGNAL_TYPE.FULLY_ACTUATED
                    || SimConfig.signalType == SimConfig.SIGNAL_TYPE.ONE_LANE_VERSION
                    || SimConfig.signalType == SimConfig.SIGNAL_TYPE.REVISED_PHASE) {
                if (SimConfig.FULLY_OBSERVING) {
//                     if (!notHinderingHumanVehicles(proposals.get(0).getArrivalLaneID(),
//                            proposals.get(0).getDepartureLaneID(),
//                            exitTime)
//                            || !notHinderingCAV(proposals.get(0).getArrivalLaneID(),
//                                    proposals.get(0).getDepartureLaneID(),
//                                    proposals.get(0).getArrivalTime())) {
                    if (!notHinderingHumanVehicles(proposals.get(0).getArrivalLaneID(),
                            proposals.get(0).getDepartureLaneID(),
                            exitTime,
                            proposals.get(0).getIntersectionManager())) {
                        basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                                Reject.Reason.NO_CLEAR_PATH);
                        return;
                    }
                } else {
                    if (!notHinderingPotentialHumanDrivers(proposals.get(0).getArrivalLaneID(),
                            proposals.get(0).getDepartureLaneID(),
                            exitTime,
                            proposals.get(0).getIntersectionManager(), Resources.vinToVehicles.get(vin).getVehicleType())) {
                        basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                                Reject.Reason.NO_CLEAR_PATH);
                        return;
                    }
                }
            }
        }

        if (reserveParam != null) {
            //these are bandaids over the issue of vehicles not yielding properly, buried somewhere above in the logic
            if (shouldYieldForPermissiveCrossTurn(proposals.get(0).getArrivalLaneID(), proposals.get(0).getDepartureLaneID(), proposals.get(0).getIntersectionManager(), reserveParam.getSuccessfulProposal().getArrivalTime(), reserveParam.getExitTime(), Resources.vinToVehicles.get(vin))) {//|| shouldYieldToForRightTurnOnRed(proposals.get(0).getArrivalLaneID(), proposals.get(0).getDepartureLaneID(), proposals.get(0).getIntersectionManager(), reserveParam.getSuccessfulProposal().getArrivalTime(), reserveParam.getExitTime(), Resources.vinToVehicles.get(vin))) {
                basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                        Reject.Reason.MUST_YIELD_FOR_TURNING_ACTION);
                return;
            }
            //this is an inheritance sin, but, ¯\_(ツ)_/¯
            if (SimConfig.signalType == SIGNAL_TYPE.FULLY_ACTUATED) {
                FullyActuatedSignalController fasc = ((FullyActuatedSignalController) signalControllers.get(reserveParam.getSuccessfulProposal().getArrivalLaneID()));
                fasc.logActuationOnLane(basePolicy.getCurrentTime(), reserveParam.getSuccessfulProposal().getArrivalTime(), reserveParam.getExitTime(), Resources.vinToVehicles.get(vin).getVehicleType());
            }
            basePolicy.sendComfirmMsg(msg.getRequestId(), reserveParam);
            DesignatedLanesExpr.laneRegistry.get(proposals.get(0).getArrivalLaneID()).exit(0);
            DesignatedLanesExpr.laneRegistry.get(proposals.get(0).getDepartureLaneID()).enter(1);
        } else {
            basePolicy.sendRejectMsg(vin, msg.getRequestId(),
                    Reject.Reason.NO_CLEAR_PATH);
        }

    }

    /**
     *
     * @param arrivalLaneID
     * @param departureLaneID
     * @param im
     * @param arrivalTime
     * @param exitTime
     * @param vehicleType
     * @return
     */
    private boolean shouldYieldForPermissiveCrossTurn(int arrivalLaneID, int departureLaneID, IntersectionManager im, double arrivalTime, double exitTime, VehicleSimView vehicle) {
        if (!SimConfig.FULLY_OBSERVING) {
            throw new RuntimeException("shouldYieldForPermissiveCrossTurn: Signal was set to not be fully observing. shouldYieldForPermissiveTurn doesn't currently allow this.");
        }

        Lane arrivalLane = DesignatedLanesExpr.laneRegistry.get(arrivalLaneID);
        Road dualRoad = arrivalLane.getContainingRoad().getDual();
        TurnDirection td = getTurnDirection(arrivalLaneID, departureLaneID);
        TrafficSignal ts = null;
        double timeUntilQueryingVehicleExits = exitTime - getCurrentTime();
        //there is a dual road, and it is turning to cross another road
        if (td == Constants.CROSS_TURN_DIRECTION && dualRoad != null) {

            ts = getRelevantTrafficSignalInTimeRange(dualRoad.getIndexLane(), arrivalTime, exitTime, EnumSet.of(TurnDirection.STRAIGHT));

            for (Lane lane : dualRoad.getLanes()) {
                //skip the lane if the lane doesn't allow straight
                Set<TurnDirection> laneAllowedTds = lane.getLaneIM().getMappedTurnDirectionsForAllVehicleTypes(im);
                if (laneAllowedTds != null && (laneAllowedTds.equals(Collections.EMPTY_LIST) || laneAllowedTds.contains(Constants.TurnDirection.STRAIGHT))) {
                    //check every vehicle in the lane
                    for (Integer vin : Resources.laneToVin.get(lane)) {
                        VehicleSimView otherVehicle = Resources.vinToVehicles.get(vin);
                        //skip the vehicle that's querying
                        if (vin != vehicle.getVIN()) {
                            //handle whether reservations or yielding should take priority
                            if (shouldVehicleTypePotentiallyYieldToVehicleTypeForFCFSSystem(vehicle.getVehicleType(), otherVehicle.getVehicleType())) {
                                double maxSpeedForApproachingVehicle = Util.getMaxSpeedForVehicleIfNotBeyondIntersectionEntrance(im, otherVehicle);
                                if (maxSpeedForApproachingVehicle == -1) {
                                    continue; //vehicle is already in or beyond the intersection, if in the intersection reservations will prevent the cars from colliding
                                }

                                //if the vehicle on the dual road is making a cross turn too, ignore it. This is a bit unrealistic, but actual drivers make a similar judgement. We're assuming unsafe operations don't get allowed by signals, here, and that AVs won't collide due to reservations..
                                if (getTurnDirection(lane.getId(), otherVehicle.getDriver().getDestination().getIndexLane().getId()) == Constants.CROSS_TURN_DIRECTION) {
                                    continue;
                                }

                                //check if approaching vehicle might have green light at incoming and/or outgoing times
                                if (ts == TrafficSignal.GREEN || ts == TrafficSignal.UNKNOWN_CONTAINING_GREEN) {
                                    if (timeUntilQueryingVehicleExits >= Util.getDistanceOfVehicleFromLaneEntrancePoint(im, otherVehicle) / maxSpeedForApproachingVehicle) {
                                        return true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        return false;
    }

    private boolean shouldVehicleTypePotentiallyYieldToVehicleTypeForFCFSSystem(SimConfig.VEHICLE_TYPE mightYield, SimConfig.VEHICLE_TYPE other) {
        if ((mightYield != SimConfig.VEHICLE_TYPE.HUMAN && mightYield != SimConfig.VEHICLE_TYPE.AUTO) || (other != SimConfig.VEHICLE_TYPE.HUMAN && other != SimConfig.VEHICLE_TYPE.AUTO)) {
            throw new RuntimeException("shouldVehicleTypePotentiallyYieldToVehicleType: Currently only human and autonomous vehicles are handled.");
        }

        if (mightYield == SimConfig.VEHICLE_TYPE.HUMAN && other == SimConfig.VEHICLE_TYPE.HUMAN) {
            return true; //human vehicles may need to yield to other human vehicles, such as in the instance of permissive left turns in the US.
        } else if (mightYield == SimConfig.VEHICLE_TYPE.HUMAN && other == SimConfig.VEHICLE_TYPE.AUTO) {
            return true; //human vehicles may need to yield to AVs, such as in the instance of permissive left turns in the US.
        } else if (mightYield == SimConfig.VEHICLE_TYPE.AUTO && other == SimConfig.VEHICLE_TYPE.HUMAN) {
            return true; //AVs vehicles may need to yield to human vehicles, such as in the instance of permissive left turns in the US.
        } else if (mightYield == SimConfig.VEHICLE_TYPE.AUTO && other == SimConfig.VEHICLE_TYPE.AUTO) {
            return false; //AVs shouldn't need to yield to other AVs with a first come first serve approval system, such as in the instance of permissive left turns in the US.
        } else {
            throw new RuntimeException("shouldVehicleTypePotentiallyYieldToVehicleType: no valid combination of vehicle types provided: " + mightYield.name() + ", " + other.name());
        }
    }

    //todo, this 2 layered search between here and the relevantSegments function is kind of redundant/ineffecient
    private TrafficSignal getRelevantTrafficSignalInTimeRange(Lane laneInOtherRoad, double arrivalTime, double exitTime, Set<TurnDirection> potentiallyConflictingTurnDirection) {
        ArrayList<RBSegmentReadOnlyNoLockingView> relevantSegments = getRelevantSegments(laneInOtherRoad, arrivalTime, exitTime, potentiallyConflictingTurnDirection);
        TrafficSignal ts = null;
        //no segment found handled the straight direction for the dual road, so assume the light will be red.
        if (relevantSegments == null) {
            ts = TrafficSignal.RED;
        } else {
            if (relevantSegments.size() == 1) {
                ts = relevantSegments.get(0).getColor();
            } else {
                boolean hasYellowOrRed = false;
                boolean hasGreen = false;
                for (RBSegmentReadOnlyNoLockingView seg : relevantSegments) {
                    if (seg.isRedOrYellowSegment()) {
                        hasYellowOrRed = true;
                    } else if (seg.getColor() == TrafficSignal.GREEN) {
                        hasGreen = true;
                    }

                    if (hasYellowOrRed && hasGreen) {
                        ts = TrafficSignal.UNKNOWN_CONTAINING_GREEN;
                    } else if (!hasYellowOrRed && hasGreen) {
                        ts = TrafficSignal.GREEN;
                    } else {
                        ts = TrafficSignal.UNKNOWN;
                    }
                }
            }
        }

        return ts;
    }

    private ArrayList<RBSegmentReadOnlyNoLockingView> getRelevantSegments(Lane laneInOtherRoad, double arrivalTime, double exitTime, Set<TurnDirection> potentiallyConflictingTurnDirections) {
        int laneId = laneInOtherRoad.getId();
        //get phase segments in the window arround arrival and exit
        List<LinkedHashSet<RBSegmentReadOnlyNoLockingView>> arrivalSegments = ((FullyActuatedSignalController) signalControllers.get(laneId)).getPhaseSegmentsAtTimePotentiallyLockingSegmentData(arrivalTime);
        List<LinkedHashSet<RBSegmentReadOnlyNoLockingView>> exitSegments = ((FullyActuatedSignalController) signalControllers.get(laneId)).getPhaseSegmentsAtTimePotentiallyLockingSegmentData(exitTime);
        //record keeping lists
        List<RBSegmentReadOnlyNoLockingView> startingSegments = new ArrayList<RBSegmentReadOnlyNoLockingView>();
        List<ArrayList<RBSegmentReadOnlyNoLockingView>> workingSegments = new ArrayList<ArrayList<RBSegmentReadOnlyNoLockingView>>();
        int relevantIndex = -1;
        //go through all the rings
        for (int i = 0; i < arrivalSegments.size(); ++i) {
            //todo, this is ineffecient, but I'm not rewriting/extending the linkedhashset right now to cache the last element for me. It's possible linkedhashset could just be replaced by a list, too.
            //get the last segment in the ring that's possible in the time window
            RBSegmentReadOnlyNoLockingView lastSegment = (RBSegmentReadOnlyNoLockingView) exitSegments.get(i).toArray()[exitSegments.get(i).size() - 1];
            //get the first segment in the ring that's possible in the time window
            RBSegmentReadOnlyNoLockingView currentSegment = arrivalSegments.get(i).iterator().next();
            //create a list and store it for record keeping
            workingSegments.add(new ArrayList<RBSegmentReadOnlyNoLockingView>());

            //add the segment as a segment to search if it is not a barrier, is for the desired road, and allows any of the desired turning actions
            if (!currentSegment.isBarrier() && currentSegment.getRoad() == laneInOtherRoad.getContainingRoad() && (currentSegment.getTurnDirectionsForPhaseSegment() == Collections.EMPTY_SET || potentiallyConflictingTurnDirections == Collections.EMPTY_SET || !Collections.disjoint(currentSegment.getTurnDirectionsForPhaseSegment(), potentiallyConflictingTurnDirections))) {
                workingSegments.get(i).add(currentSegment);
            }
            //mark the starting segment
            startingSegments.add(currentSegment);

            //flag for if this is the first loop
            boolean firstCheck = true;
            //loop until we hit the last segment in the exit range
            while (!currentSegment.equals(lastSegment)) {
                if (!firstCheck && currentSegment == startingSegments.get(i)) {
                    throw new RuntimeException("Segment lookup in shouldYieldForPermissiveTurn wrapped. This is not allowed.");
                } else {
                    //no longer on the first loop
                    firstCheck = false;
                }
                //advance the segment
                currentSegment = currentSegment.getRBSegmentReadOnlyNoLockingViewForNextPhaseSegment();
                //add the segment as a segment to search if it is not a barrier, is for the desired road, and allows any of the desired turning actions
                if (!currentSegment.isBarrier() && currentSegment.getRoad() == laneInOtherRoad.getContainingRoad() && (currentSegment.getTurnDirectionsForPhaseSegment() == Collections.EMPTY_SET || potentiallyConflictingTurnDirections == Collections.EMPTY_SET || !Collections.disjoint(currentSegment.getTurnDirectionsForPhaseSegment(), potentiallyConflictingTurnDirections))) {
                    workingSegments.get(i).add(currentSegment);
                }
            }

            //flag the proper set
            if (!workingSegments.get(i).isEmpty()) {
                if (relevantIndex == -1) {
                    relevantIndex = i;
                } else {
                    throw new RuntimeException("Multiple rings found which may service the straight direction and road in shouldYieldForPermissiveTurn. This is not allowed.");
                }
            }
        }
        //return the proper set
        return (relevantIndex == -1 ? null : workingSegments.get(relevantIndex));
    }

    private EnumMap<Constants.Direction, EnumMap<Constants.TurnDirection, ArrayList<RBSegmentReadOnlyNoLockingView>>> getAllPossibleSegmentsMappedByDirectionAndAction(FullyActuatedSignalController controller, double arrivalTime, double exitTime) {
        EnumMap<Constants.Direction, EnumMap<Constants.TurnDirection, ArrayList<RBSegmentReadOnlyNoLockingView>>> retMap = new EnumMap<Constants.Direction, EnumMap<Constants.TurnDirection, ArrayList<RBSegmentReadOnlyNoLockingView>>>(Constants.Direction.class);
        //get phase segments in the window arround arrival and exit
        List<LinkedHashSet<RBSegmentReadOnlyNoLockingView>> arrivalSegments = controller.getPhaseSegmentsAtTimePotentiallyLockingSegmentData(arrivalTime);
        List<LinkedHashSet<RBSegmentReadOnlyNoLockingView>> exitSegments = controller.getPhaseSegmentsAtTimePotentiallyLockingSegmentData(exitTime);
        //record keeping lists
        List<RBSegmentReadOnlyNoLockingView> startingSegments = new ArrayList<RBSegmentReadOnlyNoLockingView>();
        //go through all the rings
        for (int i = 0; i < arrivalSegments.size(); ++i) {
            //todo, this is ineffecient, but I'm not rewriting/extending the linkedhashset right now to cache the last element for me. It's possible linkedhashset could just be replaced by a list, too.
            //get the last segment in the ring that's possible in the time window
            RBSegmentReadOnlyNoLockingView lastSegment = (RBSegmentReadOnlyNoLockingView) exitSegments.get(i).toArray()[exitSegments.get(i).size() - 1];
            //get the first segment in the ring that's possible in the time window
            RBSegmentReadOnlyNoLockingView currentSegment = arrivalSegments.get(i).iterator().next();

            //add the segment as a segment to search
            Constants.Direction dir = (currentSegment.isBarrier() ? Util.getDirectionFromHeadingCardinal(im.getIntersection().getEntryHeading(currentSegment.getRBSegmentReadOnlyNoLockingViewForSegmentRepresentingPhase().getRoad().getIndexLane())) : Util.getDirectionFromHeadingCardinal(im.getIntersection().getEntryHeading(currentSegment.getRoad().getIndexLane())));
            if (!retMap.containsKey(dir)) {
                retMap.put(dir, new EnumMap<Constants.TurnDirection, ArrayList<RBSegmentReadOnlyNoLockingView>>(Constants.TurnDirection.class));
            }
            Set<TurnDirection> tdsForPhaseSegment = (currentSegment.isBarrier() ? currentSegment.getRBSegmentReadOnlyNoLockingViewForSegmentRepresentingPhase().getTurnDirectionsForPhaseSegment() : currentSegment.getTurnDirectionsForPhaseSegment());

            for (Constants.TurnDirection td : tdsForPhaseSegment) {
                if (!retMap.get(dir).containsKey(td)) {
                    retMap.get(dir).put(td, new ArrayList<RBSegmentReadOnlyNoLockingView>());
                }
                retMap.get(dir).get(td).add(currentSegment);
            }
            //mark the starting segment
            startingSegments.add(currentSegment);

            //flag for if this is the first loop
            boolean firstCheck = true;
            //loop until we hit the last segment in the exit range
            while (!currentSegment.equals(lastSegment)) {
                if (!firstCheck && currentSegment == startingSegments.get(i)) {
                    throw new RuntimeException("Segment lookup in shouldYieldForPermissiveTurn wrapped. This is not allowed.");
                } else {
                    //no longer on the first loop
                    firstCheck = false;
                }
                //advance the segment
                currentSegment = currentSegment.getRBSegmentReadOnlyNoLockingViewForNextPhaseSegment();
                //add the segment as a segment to search if it is not a barrier, is for the desired road, and allows any of the desired turning actions
                dir = (currentSegment.isBarrier() ? Util.getDirectionFromHeadingCardinal(im.getIntersection().getEntryHeading(currentSegment.getRBSegmentReadOnlyNoLockingViewForSegmentRepresentingPhase().getRoad().getIndexLane())) : Util.getDirectionFromHeadingCardinal(im.getIntersection().getEntryHeading(currentSegment.getRoad().getIndexLane())));

                if (!retMap.containsKey(dir)) {
                    retMap.put(dir, new EnumMap<Constants.TurnDirection, ArrayList<RBSegmentReadOnlyNoLockingView>>(Constants.TurnDirection.class));
                }
                for (Constants.TurnDirection td : (currentSegment.isBarrier() ? currentSegment.getRBSegmentReadOnlyNoLockingViewForSegmentRepresentingPhase().getTurnDirectionsForPhaseSegment() : currentSegment.getTurnDirectionsForPhaseSegment())) {
                    if (!retMap.get(dir).containsKey(td)) {
                        retMap.get(dir).put(td, new ArrayList<RBSegmentReadOnlyNoLockingView>());
                    }
                    retMap.get(dir).get(td).add(currentSegment);
                }
            }

        }
        //return the results
        return retMap;
    }

    private boolean canFollowFrontVehicle(VehicleSimView vehicle) {
        VehicleSimView frontVehicle = vehicle.getFrontVehicle();

        // if no such vehicle in front of it, he cannot follow.
        if (frontVehicle == null) {
            return false;
        }

        // if the vehicle is following the vehicle in front of it within a certain distance
        // it's okay to follow it.
        if (vehicle.getPosition().distance(frontVehicle.getPosition()) < SimConfig.FOLLOW_DISTANTCE) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TrafficSignal getSignal(int laneId) {
        return signalControllers.get(laneId).getSignal(
                basePolicy.getCurrentTime());
    }

    public TrafficSignal getSignalForGUI(int laneId) {
        SignalController sc = signalControllers.get(laneId);
        //this is an inheritance sin, but, ¯\_(ツ)_/¯
        if (sc instanceof FullyActuatedSignalController) {
            return ((FullyActuatedSignalController) signalControllers.get(laneId)).getSignalForGUI();
        } else {
            return getSignal(laneId);
        }
    }

    /////////////////////////////////
    // PRIVATE METHODS
    /////////////////////////////////
    /**
     * Check whether the vehicle can enter the intersection from a lane at the
     * current time. This method is intended to be overridden by superclass.
     *
     * THIS FUNCTION IS ONLY FOR SIGNAL, NOT FCFS-SIGNAL
     *
     * Later note, the implementation of this function allows rightmost lane to
     * rightmost lane (in the US) turns and entry to the intersection if the
     * signal is green.
     *
     * Note, this function will handle multiple lanes turning right on red, but
     * will only allow more than the rightmost lane to turn red if the
     * destination lane for that lane is not to the left of any outgoing lane
     * which has an incoming left turn or straight through action. Also, this
     * function currently assumes with traffic turns are right turns (as in the
     * United States)
     *
     * @return whether the vehicle can enter the intersection
     */
    private boolean canEnterFromLane(int arrivalLaneId, int departureLaneId, SimConfig.VEHICLE_TYPE vType, TrafficSignal sig, boolean beCautious) {
        if (Resources.map.getImRegistry().getValues().size() != 1) {
            throw new RuntimeException("One and only one intersection is supported by the canEnterFromLane function at this time, but there are: " + Resources.map.getImRegistry().getValues().size());
        }
        IntersectionManager im = Resources.map.getImRegistry().getValues().get(0);
        Lane arrivalLane = DesignatedLanesExpr.laneRegistry.get(arrivalLaneId);
        if (arrivalLane == null) {
            throw new IllegalArgumentException("Arrival lane couldn't be found in the lane registry.");
        }
        Lane departureLane = DesignatedLanesExpr.laneRegistry.get(departureLaneId);
        if (departureLane == null) {
            throw new IllegalArgumentException("Departure lane couldn't be found in the lane registry.");
        }

        TurnDirection td = getTurnDirection(arrivalLaneId, departureLaneId);

        //this being null indicates either all are allowed, or this turn direction isn't allowed
        Lane mappedDepartureLaneForActionFromArrivalLane = arrivalLane.getLaneIM().getMappedExitLane(im, td);

        //this tells us which of the above is true. Will be true if all are allowed or a specific entry to exit lane mapping has been made for the turning action.
        boolean actionIsAllowedForArrivalLaneAndVehicleType = arrivalLane.getLaneIM().isValidActionFromLane(im, td, vType);

        TrafficSignal signal = (sig == null ? getSignal(arrivalLaneId) : sig);
        //if the light is green or in some cases *might* be green and the turn action is allowed for the vehicle type
        if (actionIsAllowedForArrivalLaneAndVehicleType && ((beCautious && signal == TrafficSignal.UNKNOWN_CONTAINING_GREEN) || signal == TrafficSignal.GREEN)) {
            return true;
        } else if (SimConfig.ALLOW_RIGHT_TURNS_ON_RED_FOR_ANY_VEHICLE_TYPE && SimConfig.useExplicitMappingsForWithTrafficTurnOnRed) {
            //if a turn with traffic is permitted from the lane at all and the vehicle is explicitly allowed to turn with traffic on red from the lane
            return arrivalLane.getLaneIM().getWithTrafficTurnOnRedAllowed(im)
                    && (departureLane == mappedDepartureLaneForActionFromArrivalLane || (actionIsAllowedForArrivalLaneAndVehicleType && mappedDepartureLaneForActionFromArrivalLane == null))
                    && td == Constants.WITH_TRAFFIC_TURN_DIRECTION;
        } else {
            if (!SimConfig.ALLOW_RIGHT_TURNS_ON_RED_FOR_ANY_VEHICLE_TYPE) {
                return false;
            }
            Road departureRoad = departureLane.getContainingRoad();
            //try to figure out if a turn on red would be safe (no conflicting incoming trajectories to lanes right of the target lane)
            boolean turnShouldNotConflict = departureRoad.getOutgoingLaneByClosedLaneSkippingIndexFromRightWhichMightSupportTurnsOnRed(im, departureLane) != null
                    && departureRoad.getOutgoingLaneByClosedLaneSkippingIndexFromRightWhichMightSupportTurnsOnRed(im, departureLane).equals(arrivalLane.getContainingRoad().getIncomingLaneByClosedLaneSkippingIndexFromRightWhichMightSupportTurnsOnRed(im, arrivalLane));
            //turn is rightmost lane to rightmost lane, typical of turns on red
            return td == Constants.WITH_TRAFFIC_TURN_DIRECTION && turnShouldNotConflict;
        }

        //old code
        /*if (getSignalFromSegmentRange(arrivalLaneId) == TrafficSignal.GREEN) {
            return true;
        } else {
            // TODO: should not hard code it: need to make it more general
            if (arrivalLaneId == 2 && departureLaneId == 8) {
                return true;
            } else if (arrivalLaneId == 11 && departureLaneId == 2) {
                return true;
            } else if (arrivalLaneId == 5 && departureLaneId == 11) {
                return true;
            } else if (arrivalLaneId == 8 && departureLaneId == 5) {
                return true;
            } else {
                return false;
            }
        }*/
    }

    private TurnDirection getTurnDirection(int arrivalLaneId, int departureLaneId) {
        boolean isLeftTurn = makingLeftTurn(arrivalLaneId, departureLaneId);
        boolean isRightTurn = makingRightTurn(arrivalLaneId, departureLaneId);
        if (isLeftTurn) {
            return Constants.TurnDirection.LEFT;
        } else if (isRightTurn) {
            return Constants.TurnDirection.RIGHT;
        } else {
            return Constants.TurnDirection.STRAIGHT;
        }
        //uturns aren't allowed at the moment
    }

    /////////////////////////////////
    // PRIVATE METHODS
    /////////////////////////////////
    /**
     * Check whether the vehicle can enter the intersection from a lane at the
     * ARRIVAL time. For human drivers, we need to check this. Currently only
     * handles roads that enter the intersection in cardinal directions.
     *
     * @param arrivalLaneId the id of the lane from which the vehicle enters the
     * intersection.
     * @return whether the vehicle can enter the intersection
     */
    private boolean canEnterFromLaneAtTimepoint(int arrivalLaneId,
            int departureLaneId,
            double arrivalTime,
            IntersectionManager im,
            SimConfig.VEHICLE_TYPE vType,
            boolean beCautious) {
        return canEnterFromLaneAtTimepoint(arrivalLaneId, departureLaneId, arrivalTime, im, vType, beCautious, null);
    }

    private boolean canEnterFromLaneAtTimepoint(int arrivalLaneId,
            int departureLaneId,
            double arrivalTime,
            IntersectionManager im,
            SimConfig.VEHICLE_TYPE vType,
            boolean beCautious,
            TurnDirection tdForVerification) {
        if (im == null) {
            throw new RuntimeException("IM cannot be null in canEnterFromLaneAtTimepoint, as intersection topology and turning restrictions may affect the decision.");
        }

        Lane arrivalLane = Debug.currentMap.getLaneRegistry().get(arrivalLaneId);
        Constants.Direction arrivalLaneDir = Util.getDirectionFromHeadingCardinal(im.getIntersection().getEntryHeading(arrivalLane));
        Lane departureLane = Debug.currentMap.getLaneRegistry().get(departureLaneId);
        Constants.Direction departureLaneDir = Util.getDirectionFromHeadingCardinal(im.getIntersection().getEntryHeading(departureLane));
        boolean isCrossTurn = makingLeftTurn(arrivalLaneId, departureLaneId);
        if (departureLane != null) {
            Set<Constants.TurnDirection> turnDirs = arrivalLane.getLaneIM().validActionsFromLane(im, vType);
            //check if all turn directions are allowed or the departure lane is the mapped lane for a cross turn from the incoming lane
            isCrossTurn = (turnDirs != null && turnDirs.equals(Collections.EMPTY_SET) && Util.isCrossTurnForCardinalDirectionRoads(arrivalLaneDir, departureLaneDir)) || (turnDirs != null && turnDirs.contains(Constants.CROSS_TURN_DIRECTION) && arrivalLane.getLaneIM().getMappedExitLane(im, Constants.CROSS_TURN_DIRECTION) == departureLane);
        }

        TrafficSignal ts = null;
        //again, this is an inheritance sin, but, ¯\_(ツ)_/¯
        if (SimConfig.signalType == SIGNAL_TYPE.FULLY_ACTUATED) {
            /*if (isCrossTurn) {
                ts = ((FullyActuatedSignalController) signalControllers.get(arrivalLaneId)).getSignal(arrivalTime, EnumSet.of(Constants.CROSS_TURN_DIRECTION));
            } else {*/
            ts = ((FullyActuatedSignalController) signalControllers.get(arrivalLaneId)).getSignal(arrivalTime, EnumSet.of(getTurnDirection(arrivalLaneId, departureLaneId)));
            //}
        } else {
            ts = signalControllers.get(arrivalLaneId).getSignal(arrivalTime);
        }

        //check if the traffic signal applies to this vehicle type. For instance, if an AV turns left from a lane that never allows lefts for HVs, the signal doesn't ever matter. This check is done by seeing is the turning direction is used by humans. If the vehicle is human, no problem. If it's an AV, we'll fail.
        if (!arrivalLane.getLaneIM().isValidActionFromLane(im, getTurnDirection(arrivalLaneId, departureLaneId), SimConfig.VEHICLE_TYPE.HUMAN)) {
            return false;
        }

        if (isCrossTurn && (SimConfig.ALWAYS_ALLOW_TURNING_LEFT || (beCautious && ts == TrafficSignal.UNKNOWN_CONTAINING_GREEN) || ts == TrafficSignal.GREEN)) {
            // forbid vehicles turning left in when green lights of two opposite directions are on unless permissive turns are allowed
            return true;
        } else if (tdForVerification == null || (tdForVerification == getTurnDirection(arrivalLaneId, departureLaneId))) {
            return !isCrossTurn && canEnterFromLane(arrivalLaneId, departureLaneId, vType, ts, beCautious);
        } else {
            return false;
        }
    }

    private boolean makingLeftTurn(int arrivalLaneId, int departureLaneId) {

        if (Resources.map.getImRegistry().getValues().size() != 1) {
            throw new RuntimeException("One and only one intersection is supported by the makingLeftTurn function at this time, but there are: " + Resources.map.getImRegistry().getValues().size());
        }
        IntersectionManager im = Resources.map.getImRegistry().getValues().get(0);
        Lane arrivalLane = Debug.currentMap.getLaneRegistry().get(arrivalLaneId);
        Lane departureLane = Debug.currentMap.getLaneRegistry().get(departureLaneId);
        double arrivalHeading = im.getIntersection().getEntryHeading(arrivalLane);
        double departureHeading = im.getIntersection().getEntryHeading(departureLane);

        if (arrivalHeading % (Math.PI / 2) != 0 || departureHeading % (Math.PI / 2) != 0) {
            throw new RuntimeException("makingLeftTurn only supports roads that go in cardinal directions at the moment (intersection entry headings are even multiples of Pi/2 radians). Headings were...arrival: " + arrivalHeading + ", departure heading: " + departureHeading);
        }

        Constants.Direction arrivalLaneDir = Util.getDirectionFromHeadingCardinal(arrivalHeading);
        Constants.Direction departureLaneDir = Util.getDirectionFromHeadingCardinal(departureHeading);

        return Util.isCrossTurnForCardinalDirectionRoads(arrivalLaneDir, departureLaneDir);
    }

    private boolean makingRightTurn(int arrivalLaneId, int departureLaneId) {

        if (Resources.map.getImRegistry().getValues().size() != 1) {
            throw new RuntimeException("One and only one intersection is supported by the makingRightTurn function at this time, but there are: " + Resources.map.getImRegistry().getValues().size());
        }
        IntersectionManager im = Resources.map.getImRegistry().getValues().get(0);
        Lane arrivalLane = Debug.currentMap.getLaneRegistry().get(arrivalLaneId);
        Lane departureLane = Debug.currentMap.getLaneRegistry().get(departureLaneId);
        double arrivalHeading = im.getIntersection().getEntryHeading(arrivalLane);
        double departureHeading = im.getIntersection().getEntryHeading(departureLane);

        if (arrivalHeading % (Math.PI / 2) != 0 || departureHeading % (Math.PI / 2) != 0) {
            throw new RuntimeException("makingRightTurn only supports roads that go in cardinal directions at the moment (intersection entry headings are even multiples of Pi/2 radians). Headings were...arrival: " + arrivalHeading + ", departure heading: " + departureHeading);
        }

        Constants.Direction arrivalLaneDir = Util.getDirectionFromHeadingCardinal(arrivalHeading);
        Constants.Direction departureLaneDir = Util.getDirectionFromHeadingCardinal(departureHeading);

        return Util.isWithTrafficTurnForCardinalDirectionRoads(arrivalLaneDir, departureLaneDir);
    }

    //todo, this funciton's name doesn't match with its implementation
    /**
     *
     * @param arrivalLaneId
     * @param departureLaneId
     * @return
     */
    private boolean inRightLane(int arrivalLaneId, int departureLaneId) {
        throw new UnsupportedOperationException("This function appears to have an error in it. Please correct its behavior before use.");
        //this multiline comment was what was most recent commented out. The stuff at the bottom was commented out way before it.
        /*int arrivalRoad = Debug.currentMap.getRoad(arrivalLaneId).getIndex();
        int departureRoad = Debug.currentMap.getRoad(departureLaneId).getIndex();
        switch (arrivalRoad) {
            case 0:
                return departureRoad == 2;
            case 1:
                return departureRoad == 3;
            case 2:
                return departureRoad == 1;
            case 3:
                return departureRoad == 0;
            default:
                assert false : "Can only handle one intersection";
        }
        return false;*/

//        return arrivalLaneId == 8
//                || arrivalLaneId == 2
//                || arrivalLaneId == 11
//                || arrivalLaneId == 5;
    }

    /**
     * Check whether a certain autonomous vehicle would intersects ANY green
     * lane path. This is assuming the intersection has no idea of the coming of
     * human drivers.
     *
     * @param arrivalLaneID
     * @param departureLaneID
     * @param arrivalTime
     * @param im
     * @return
     */
    private boolean notHinderingPotentialHumanDrivers(int arrivalLaneID, int departureLaneID, double arrivalTime, IntersectionManager im, SimConfig.VEHICLE_TYPE vType) {

        Registry<Lane> laneRegistry = Resources.map.getLaneRegistry();

        for (Lane lane : laneRegistry.getValues()) {
            // only consider green lanes or uncertain lanes
            TrafficSignal ts = signalControllers.get(lane.getId()).getSignal(arrivalTime);
            if (ts != TrafficSignal.GREEN && ts != TrafficSignal.UNKNOWN_CONTAINING_GREEN) {
                continue;
            }

            List<Road> destinationRoad = Resources.destinationSelector.getPossibleDestination(lane);

            if (destinationRoad.size() == 0) {
                System.err.println("Possible destination empty in notHinderingHumanVehicles! This cannot be true.");
                System.err.println("Lane id: " + lane.getId());
            }

            for (Road road : destinationRoad) {
                for (Lane destinationLane : road.getLanes()) {
                    // Check whether it's going into the right lane.
                    // Actually, the vehicle must go into its corresponding lane the in destination road
                    if (lane.getIndexInRoad() == destinationLane.getIndexInRoad()
                            && canEnterFromLaneAtTimepoint(lane.getId(), destinationLane.getId(), arrivalTime, im, vType, true)) {
                        if (GridMapUtil.laneIntersect(arrivalLaneID, departureLaneID,
                                lane.getId(), destinationLane.getId())) {
                            // There's a chance that this vehicle would collide into the human vehicle
                            return false;
                        }
                    }
                }
            }
        }

        return true;
    }

    private boolean canEnterFromLaneAtTimepoint(VehicleSimView vehicle, double arrivalTime, IntersectionManager im) {
        if (vehicle == null || vehicle.getDriver().getState() == State.V2I_CLEARING || vehicle.getDriver().getState() == State.V2I_TERMINAL_STATE) {
            return true;
        }

        if (vehicle.getDriver().getState() == State.V2I_TRAVERSING) {
            return true;
        }

        if (!vehicle.isHuman()) {
            return canEnterFromLaneAtTimepoint(vehicle.getFrontVehicle(), arrivalTime, im);
        }

        Road currentRoad = Debug.currentMap.getRoad(vehicle.getDriver().getCurrentLane());

        //note, this lookup only works on roads in cardinal directions (N, S, E, W) due to the current implementation of the downstream functions
        //also note, we DON'T know the direction the human vehicle is going. This is a workaround for the note below (***) where it's used.
        TurnDirection td = getTurnDirection(vehicle.getDriver().getCurrentLane().getId(), vehicle.getDriver().getDestination().getIndexLane().getId());

        //int i = arrivalLane.getIndexInRoad();
        // we only consider the vehicles that are going into the intersection
        List<Lane> potentialLanes = getPotentialDestinationLanesForVehicle(vehicle, im);

        for (Lane lane : potentialLanes) {
            if (vehicle.getDriver().getState() == State.V2I_TRAVERSING) {
                return canEnterFromLaneAtTimepoint(vehicle.getFrontVehicle(), arrivalTime, im);
            }
            //***adding the verification td helps to handle case where AV is sitting in a left turn because an HV is in a right and straight lane at a red light, so the HV can technically enter the intersection going right but it wants to go straight
            //this is a quick hack to try to allow the AV to go ahead. May not be totally realistic
            if (canEnterFromLaneAtTimepoint(vehicle.getDriver().getCurrentLane().getId(), lane.getId(), arrivalTime, im, vehicle.getVehicleType(), true, td)) {
                return canEnterFromLaneAtTimepoint(vehicle.getFrontVehicle(), arrivalTime, im);
            }

        }
        return false;
    }

    private List<Lane> getPotentialDestinationLanesForVehicle(VehicleSimView vehicle, IntersectionManager im) {
        Set<TurnDirection> allowedTurnDirections = vehicle.getDriver().getCurrentLane().getLaneIM().validActionsFromLane(im, vehicle.getVehicleType());
        List<Lane> destinationLanes = new LinkedList<Lane>();
        if (allowedTurnDirections != null && allowedTurnDirections.size() > 0) {
            allowedTurnDirections = EnumSet.copyOf(allowedTurnDirections);
            allowedTurnDirections.removeAll(Arrays.asList(Constants.ALL_COMPOSED_ACTION_LABELS));

            for (TurnDirection td : allowedTurnDirections) {
                destinationLanes.add(vehicle.getDriver().getCurrentLane().getLaneIM().getMappedExitLane(im, td));
            }
        } else if (allowedTurnDirections == Collections.EMPTY_SET) {
            for (Road rd : im.getIntersection().getExitRoads()) {
                for (Lane ln : rd.getLanes()) {
                    if (im.getLanesWhichHaveAnExitLaneMappingByAnyActionAndAnyVehicleTypeToLane(ln).contains(vehicle.getDriver().getCurrentLane())) {
                        Lane currentLane = vehicle.getDriver().getCurrentLane();
                        if (currentLane == ln
                                || currentLane.getLaneIM().isValidActionFromLane(im, getTurnDirection(currentLane.getId(), ln.getId()), vehicle.getVehicleType())) {
                            destinationLanes.add(ln);
                        }
                    }
                }
            }
        } else {
            //this lane is closed, so, how did we get here?
            throw new RuntimeException("canEnterFromLaneAtTimepoint was directed to check a lane which has allowed turn directions of: null");
        }
        return destinationLanes;
    }

    /**
     * Check whether a certain autonomous vehicle would collides into a
     * potential human driver
     *
     * @param arrivalLaneID of a certain auto vehicle
     * @param departureLaneID of a certain auto vehicle
     * @param arrivalTime of a certain auto vehicle
     * @param exitTime of a certain auto vehicle
     * @param im intersection manager handling the request
     * @return whether it would collides into a human driver
     */
    private boolean notHinderingHumanVehiclesForTimeRange(int arrivalLaneID, int departureLaneID, double arrivalTime, double exitTime, IntersectionManager im, boolean myLeftIsOnRedOrYellow) {
        Map<Integer, VehicleSimView> vinToVehicles = Resources.vinToVehicles;
        //using the arrival ID doesn't matter, as the R&B uses the same "controller" for all signals by referencing the ring and barrier.
        FullyActuatedSignalController controller = (FullyActuatedSignalController) signalControllers.get(arrivalLaneID);

        EnumMap<Constants.Direction, EnumMap<Constants.TurnDirection, ArrayList<RBSegmentReadOnlyNoLockingView>>> segments = getAllPossibleSegmentsMappedByDirectionAndAction(controller, arrivalTime, exitTime);
        EnumMap<Constants.Direction, EnumMap<Constants.TurnDirection, TrafficSignal>> signals = new EnumMap<Constants.Direction, EnumMap<Constants.TurnDirection, TrafficSignal>>(Constants.Direction.class);

        for (VehicleSimView vehicle : vinToVehicles.values()) {
            if (!vehicle.isHuman()) {
                continue;
            }

            Driver driver = vehicle.getDriver();

            // if this vehicle has already left the intersection, forget it.
            if (driver.getState() == State.V2I_CLEARING || driver.getState() == State.V2I_TERMINAL_STATE) {
                continue;
            }

            //If behind the AV in question
            if (driver.getCurrentLane().getId() == arrivalLaneID && vehicle.getDriver().getState() != State.V2I_TRAVERSING && vehicle.getDriver().getState() != State.V2I_MAINTAINING_RESERVATION) {
                continue;
            }

            //if vehicle can't arrive at the intersection before the requesting AV exits, ignore it. Make sure it's not in the intersection and so it might need to be considered.
            if (!Util.vehicleCouldPossiblyArriveWithinTime(im, vehicle, exitTime) && vehicle.getDriver().getState() != State.V2I_TRAVERSING && vehicle.getDriver().getState() != State.V2I_MAINTAINING_RESERVATION) {
                continue;
            }

            Lane humanArrivalLane = (driver.getEntryLane() == null ? driver.getCurrentLane() : driver.getEntryLane());
            // In this simulator, we DO know where the human vehicle is going.
            // This is not a realistic assumption -
            // we don't know the exact destination lane in the real world.
            // So, we can only determine by the current lane of human driver

            //note, this lookup only works on roads in cardinal directions (N, S, E, W) due to the current implementation of the downstream functions
            TurnDirection humanTd = getTurnDirection(humanArrivalLane.getId(), vehicle.getDriver().getDestination().getIndexLane().getId());
            Set<Lane> humanDestinationLanes = new HashSet<Lane>();
            if (humanArrivalLane.getLaneIM().isValidActionFromLane(im, humanTd, vehicle.getVehicleType())) {
                for (TurnDirection potentialHumanTd : humanArrivalLane.getLaneIM().validActionsFromLane(im, VEHICLE_TYPE.HUMAN)) {
                    Lane ln = humanArrivalLane.getLaneIM().getMappedExitLane(im, potentialHumanTd);
                    if (ln != null) {
                        humanDestinationLanes.add(ln);
                    }
                }
            } else {
                throw new RuntimeException("Vehicle type not allowed to take desired action from lane, discovered in notHinderingHumanVehiclesForTimeRange: " + vehicle.getVehicleType().name() + " " + humanTd.name() + " on road: " + humanArrivalLane.getContainingRoad().getName() + " on lane with index " + humanArrivalLane.getContainingRoad().getRelativeIndexOfLaneInRoad(humanArrivalLane));
            }

            for (Lane lane : humanDestinationLanes) {
                TurnDirection humanTdIfTurningToLane = getTurnDirection(humanArrivalLane.getId(), lane.getId());
                if ((myLeftIsOnRedOrYellow && humanTdIfTurningToLane == Constants.CROSS_TURN_DIRECTION) || GridMapUtil.laneIntersect(arrivalLaneID, departureLaneID,
                        humanArrivalLane.getId(), lane.getId())) {

                    Constants.Direction dir = Util.getDirectionFromHeadingCardinal(im.getIntersection().getEntryHeading(humanArrivalLane));
                    Road humanArrivalRoad = (vehicle.getDriver().getEntryLane() == null ? vehicle.getDriver().getCurrentLane().getContainingRoad() : vehicle.getDriver().getEntryLane().getContainingRoad());
                    if ((myLeftIsOnRedOrYellow && humanTdIfTurningToLane == Constants.CROSS_TURN_DIRECTION) && humanArrivalRoad == DesignatedLanesExpr.laneRegistry.get(arrivalLaneID).getContainingRoad()) {
                        continue;
                    }

                    if (vehicle.getDriver().getState() == State.V2I_TRAVERSING || vehicle.getDriver().getState() == State.V2I_MAINTAINING_RESERVATION) {
                        //the vehicle is in the intersection right now or is intending to enter the intersection, as we're requesting the reservation, so we need to check if it will be gone by the time we enter. Since it's human, we assume the end of yellow of the current traffic light is the latest it can be there.
                        //check if any segments between the CAV's arrival and departure handle the turn direction
                        if (segments.get(dir) != null) {
                            Double latestPossibleTimeTheAssociatedYellowCouldExpire = Resources.ringAndBarrier.getLatestEndOfYellowOrRedForRingWithCurrentSegmentHandlingTurnDirectionForRoadAndLockIfNeeded(humanArrivalLane.getContainingRoad(), humanTdIfTurningToLane);
                            if (latestPossibleTimeTheAssociatedYellowCouldExpire != null) {
                                if (arrivalTime <= latestPossibleTimeTheAssociatedYellowCouldExpire) {
                                    return false; //vehicle could still be in the intersection by the time the CAV arrives
                                }
                            }
                        }
                    }

                    //TODO, check if the vehicle can arrive in time for the green
                    if (signals.get(dir) == null) {
                        signals.put(dir, new EnumMap<TurnDirection, TrafficSignal>(TurnDirection.class));
                    }

                    if (signals.get(dir).get(humanTdIfTurningToLane) == null && segments.get(dir) != null) {
                        TrafficSignal ts = null;
                        ArrayList<RBSegmentReadOnlyNoLockingView> relevantSegments = segments.get(dir).get(humanTdIfTurningToLane);
                        //no segment found handled the straight direction for the dual road, so assume the light will be red.
                        if (relevantSegments == null) {
                            ts = TrafficSignal.RED;
                        } else {
                            if (relevantSegments.size() == 1) {
                                ts = relevantSegments.get(0).getColor();
                            } else {
                                boolean hasYellowOrRed = false;
                                boolean hasRed = false;
                                boolean hasYellow = false;
                                boolean hasGreen = false;
                                for (RBSegmentReadOnlyNoLockingView seg : relevantSegments) {
                                    if (seg.getColor() == TrafficSignal.RED) {
                                        hasRed = true;
                                    } else if (seg.getColor() == TrafficSignal.YELLOW) {
                                        hasYellow = true;
                                    } else if (seg.getColor() == TrafficSignal.GREEN) {
                                        hasGreen = true;
                                    }

                                    if (hasYellow && hasGreen) {
                                        ts = TrafficSignal.UNKNOWN_CONTAINING_GREEN_AND_YELLOW;
                                    } else if (hasYellow && !hasGreen) {
                                        ts = TrafficSignal.UNKNOWN_CONTAINING_YELLOW;
                                    } else if (!hasYellow && !hasRed && hasGreen) {
                                        ts = TrafficSignal.GREEN;
                                    } else if (!hasYellow && hasGreen) {
                                        ts = TrafficSignal.UNKNOWN_CONTAINING_GREEN;
                                    } else {
                                        ts = TrafficSignal.UNKNOWN;
                                    }
                                }
                            }
                        }
                        signals.get(dir).put(humanTdIfTurningToLane, ts);
                    }

                    if (signals.get(dir).get(humanTdIfTurningToLane) != null
                            && (signals.get(dir).get(humanTdIfTurningToLane) == TrafficSignal.UNKNOWN_CONTAINING_GREEN_AND_YELLOW /*|| signals.get(dir).get(humanTdIfTurningToLane) == TrafficSignal.UNKNOWN_CONTAINING_YELLOW*/ || signals.get(dir).get(humanTdIfTurningToLane) == TrafficSignal.UNKNOWN_CONTAINING_GREEN || signals.get(dir).get(humanTdIfTurningToLane) == TrafficSignal.GREEN)) {
                        // The approaching vehicle crosses a potentially active green trajectory
                        return false;
                    }
                }
            }
        }

        return true;
    }

    /**
     * Check whether a certain autonomous vehicle would collides into a
     * potential human driver
     *
     * @param arrivalLaneID of a certain auto vehicle
     * @param departureLaneID of a certain auto vehicle
     * @param arrivalTime of a certain auto vehicle
     * @param im intersection manager handling the request
     * @return whether it would collides into a human driver
     */
    private boolean notHinderingHumanVehicles(int arrivalLaneID, int departureLaneID, double arrivalTime, IntersectionManager im) {
        Map<Integer, VehicleSimView> vinToVehicles = Resources.vinToVehicles;

//         if (true) {
//            return true;
//        }
        for (VehicleSimView vehicle : vinToVehicles.values()) {
            if (!vehicle.isHuman()) {
                continue;
            }

            Driver driver = vehicle.getDriver();

            // if this vehicle has already left the intersection, forget it.
            if (driver.getState() == State.V2I_CLEARING || driver.getState() == State.V2I_TERMINAL_STATE) {
                continue;
            }

            //If behind the AV in question
            if (driver.getCurrentLane().getId() == arrivalLaneID) {
                continue;
            }
            // if it's in green light, and it's human,
            // then the path of this vehicle must not intersect with the human
            if (canEnterFromLaneAtTimepoint(vehicle, arrivalTime, im)) {
                //  if (vehicle.isHuman() || vehicle.withCruiseControll() || vehicle.withAdaptiveCruiseControll()) { Guni: Changed here
                //^^^
                /*Lane humanArrivalLane = driver.getCurrentLane();
                // In this simulator, we DO know where the human vehicle is going.
                // This is not a realistic assumption -
                // we don't know the exact destination lane in the real world.
                // So, we can only determine by the current lane of human driver

                //note, this lookup only works on roads in cardinal directions (N, S, E, W) due to the current implementation of the downstream functions
                TurnDirection humanTd = getTurnDirection(humanArrivalLane.getId(), vehicle.getDriver().getDestination().getIndexLane().getId());
                Set<Lane> humanDestinationLanes = new HashSet<Lane>();

                if (humanArrivalLane.getLaneIM().isValidActionFromLane(im, humanTd, vehicle.getVehicleType())) {
                    for (TurnDirection potentialHumanTd : humanArrivalLane.getLaneIM().validActionsFromLane(im, VEHICLE_TYPE.HUMAN)) {
                        Lane ln = humanArrivalLane.getLaneIM().getMappedExitLane(im, potentialHumanTd);
                        if (ln != null) {
                            humanDestinationLanes.add(ln);
                        }
                    }
                } else {
                    humanTd = getTurnDirection(driver.getEntryLane().getId(), vehicle.getDriver().getDestination().getIndexLane().getId());
                    if (humanArrivalLane != driver.getEntryLane() && driver.getEntryLane().getLaneIM().isValidActionFromLane(im, humanTd, vehicle.getVehicleType())) {
                        //the vehicle has just completed a turn onto a lane which it could not have gone onto if had been going straight through the intersection from that lane's incoming road, but the turn was valid nonetheless. So we fudge this a bit. See note "TODO, uncomment below, test and then use it instead of lanes from ^^^ to here." below
                        for (TurnDirection potentialHumanTd : humanArrivalLane.getLaneIM().validActionsFromLane(im, VEHICLE_TYPE.HUMAN)) {
                            Lane ln = humanArrivalLane.getLaneIM().getMappedExitLane(im, potentialHumanTd);
                            if (ln != null) {
                                humanDestinationLanes.add(ln);
                            }
                        }
                        humanDestinationLanes.add(humanArrivalLane);
                    } else {
                        throw new RuntimeException("Vehicle type not allowed to take desired action from lane, discovered in notHinderingHumanVehicles: " + vehicle.getVehicleType().name() + " " + humanTd.name() + " on road: " + humanArrivalLane.getContainingRoad().getName() + " on lane with index " + humanArrivalLane.getContainingRoad().getRelativeIndexOfLaneInRoad(humanArrivalLane));
                    }
                }*/
                //TODO, uncomment below, test and then use it instead of lanes from ^^^ to here. The below wasn't implemented for consistency with some experiements, but is the way the check should be done.
                Lane humanArrivalLane = (driver.getEntryLane() == null ? driver.getCurrentLane() : driver.getEntryLane());
                // In this simulator, we DO know where the human vehicle is going.
                // This is not a realistic assumption -
                // we don't know the exact destination lane in the real world.
                // So, we can only determine by the current lane of human driver

                //note, this lookup only works on roads in cardinal directions (N, S, E, W) due to the current implementation of the downstream functions
                TurnDirection humanTd = getTurnDirection(humanArrivalLane.getId(), vehicle.getDriver().getDestination().getIndexLane().getId());
                Set<Lane> humanDestinationLanes = new HashSet<Lane>();
                if (humanArrivalLane.getLaneIM().isValidActionFromLane(im, humanTd, vehicle.getVehicleType())) {
                    for (TurnDirection potentialHumanTd : humanArrivalLane.getLaneIM().validActionsFromLane(im, VEHICLE_TYPE.HUMAN)) {
                        Lane ln = humanArrivalLane.getLaneIM().getMappedExitLane(im, potentialHumanTd);
                        if (ln != null) {
                            humanDestinationLanes.add(ln);
                        }
                    }
                } else {
                    throw new RuntimeException("Vehicle type not allowed to take desired action from lane, discovered in notHinderingHumanVehicles: " + vehicle.getVehicleType().name() + " " + humanTd.name() + " on road: " + humanArrivalLane.getContainingRoad().getName() + " on lane with index " + humanArrivalLane.getContainingRoad().getRelativeIndexOfLaneInRoad(humanArrivalLane));
                }

                for (Lane lane : humanDestinationLanes) {
                    if (GridMapUtil.laneIntersect(arrivalLaneID, departureLaneID,
                            humanArrivalLane.getId(), lane.getId())) {
                        // The approaching vehicle crosses an active green trajectory
                        return false;
                    }
                }
            }
        }

        return true;
    }

//    /**
//     * Check whether a certain autonomous vehicle would collides into a CAV with
//     * green light
//     *
//     * @param arrivalLaneID of a certain auto vehicle
//     * @param departureLaneID of a certain auto vehicle
//     * @param arrivalTime of a certain auto vehicle
//     * @return whether it would collides into a human driver
//     */
//    private boolean notHinderingCAV(int arrivalLaneID, int departureLaneID, double arrivalTime) {
//
//        Map<Integer, VehicleSimView> vinToVehicles = Resources.vinToVehicles;
//
////             if (true) {
////            return true;
////        }
//        for (VehicleSimView vehicle : vinToVehicles.values()) {
//            if (vehicle.isHuman()) {
//                continue;
//            }
//            Driver driver = vehicle.getDriver();
//
//            // if this vehicle has already left the intersection, forget it.
//            if (driver.getState() == State.V2I_CLEARING) {
//                continue;
//            }
//
//            List<Lane> destinationLanes = driver.getDestination().getLanes();
//
//            // we only consider the vehicles that are going into the intersection
//            boolean canGetInto = false;
//            for (Lane lane : destinationLanes) {
//                if (canEnterFromLaneAtTimepoint(driver.getCurrentLane().getId(), lane.getId(), arrivalTime)) {
//                    canGetInto = true;
//                    break;
//                }
//            }
//
//            // if it's in green light, and it's CAV,
//            // then the path of this vehicle must not intersect with the CAV
//            if (canGetInto) {
//
//                // In this simulator, we DO know where the human vehicle is going.
//                // This is not a realistic assumption -
//                // we don't know the exact destination lane in the real world.
//                // So, we can only determine by the current lane of human driver
//                Lane destinationLane = driver.getDestination().getLanes().get(driver.getCurrentLane().getIndexInRoad());
//
//			  			// Check whether it's going into the right lane.
//                // Actually, the vehicle must go into its corresponding lane the in destination road
//                if (GridMapUtil.laneIntersect(arrivalLaneID, departureLaneID,
//                        driver.getCurrentLane().getId(), destinationLane.getId())) {
//                    // There's a chance that this vehicle would collide into the human vehicle
//                    return false;
//                }
//
//            }
//        }
//
//        return true;
//    }
//    /**
//     * Check whether a certain autonomous vehicle would collides into a
//     * potential human driver
//     *
//     * @param arrivalLaneID of a certain auto vehicle
//     * @param departureLaneID of a certain auto vehicle
//     * @param arrivalTime of a certain auto vehicle
//     * @return whether it would collides into a human driver
//     */
//    private boolean notHinderingHumanVehicles(int arrivalLaneID, int departureLaneID, double arrivalTime) {
//        Map<Integer, VehicleSimView> vinToVehicles = Resources.vinToVehicles;
//
////         if (SimConfig.FULLY_OBSERVING == true) {
////            return true;
////        }
//         
//        for (VehicleSimView vehicle : vinToVehicles.values()) {
//            Driver driver = vehicle.getDriver();
//
//            // if this vehicle has already left the intersection, forget it.
//            if (driver.getState() == State.V2I_CLEARING) {
//                continue;
//            }
//
//            List<Lane> destinationLanes = driver.getDestination().getLanes();
//
//            // we only consider the vehicles that are going into the intersection
//            boolean canGetInto = false;
//            for (Lane lane : destinationLanes) {
//                if (canEnterFromLaneAtTimepoint(driver.getCurrentLane().getId(), lane.getId(), arrivalTime)) {
//                    canGetInto = true;
//                    break;
//                }
//            }
//
//            // if it's in green light, and it's human,
//            // then the path of this vehicle must not intersect with the human
//            if (canGetInto) {
//                if (vehicle.isHuman() || vehicle.withCruiseControll() || vehicle.withAdaptiveCruiseControll()) {
//                    Lane humanArrivalLane = driver.getCurrentLane();
//
//                    // In this simulator, we DO know where the human vehicle is going.
//                    // This is not a realistic assumption -
//                    // we don't know the exact destination lane in the real world.
//                    // So, we can only determine by the current lane of human driver
//                    List<Road> destinationRoad = Resources.destinationSelector.getPossibleDestination(humanArrivalLane);
//
//                    if (destinationRoad.size() == 0) {
//                        System.err.println("Possible destination empty in notHinderingHumanVehicles! This cannot be true.");
//                    }
//
//                    for (Road road : destinationRoad) {
//                        for (Lane lane : road.getLanes()) {
//                            // Check whether it's going into the right lane.
//                            // Actually, the vehicle must go into its corresponding lane the in destination road
//                            if (lane.getIndexInRoad() == humanArrivalLane.getIndexInRoad()) {
//                                if (GridMapUtil.laneIntersect(arrivalLaneID, departureLaneID,
//                                        humanArrivalLane.getId(), lane.getId())) {
//                                    // There's a chance that this vehicle would collide into the human vehicle
//                                    return false;
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }
//
//        return true;
//    }
    /**
     * Check whether all the vehicles in the green lanes have get reservation.
     * If so, the vehicles in red lanes can feel free to send request
     *
     * THIS IS NOT A GOOD WAY TO DETERMINE WHETHER A AUTO VEHICLE CAN GO ACCROSS
     *
     * @return whether all get reservation or not
     *
     */
    private boolean allHumanVehicleInGreenLaneGetReservation(double arrivalTime, IntersectionManager im) {
        Map<Integer, VehicleSimView> vinToVehicles = Resources.vinToVehicles;

        for (VehicleSimView vehicle : vinToVehicles.values()) {
            Driver driver = vehicle.getDriver();

            List<Lane> destinationLanes = driver.getDestination().getLanes();

            boolean canGetInto = false;
            for (Lane lane : destinationLanes) {
                if (canEnterFromLaneAtTimepoint(driver.getCurrentLane().getId(), lane.getId(), arrivalTime, im, vehicle.getVehicleType(), false)) {
                    // in this case, this vehicle should go into intersection, if it's driven by human
                    canGetInto = true;
                    break;
                }
            }

            // if it's in green light, and it's human, then it must have reservation
            // of course, it cannot be a human driver
            if (canGetInto) {
                if (vehicle.isHuman()) {
                    int vin = vehicle.getVIN();
                    if (!basePolicy.hasReservation(vin)) {
                        // a human driver has not got a reservation here
                        return false;
                    }
                }
            }
        }

        return true;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public StatCollector<?> getStatCollector() {
        return null;
    }

}
