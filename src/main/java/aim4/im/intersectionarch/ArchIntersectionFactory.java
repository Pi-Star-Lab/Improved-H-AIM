package aim4.im.intersectionarch;

import aim4.config.Constants;
import aim4.config.SimConfig;
import aim4.util.Util;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

public class ArchIntersectionFactory {

    private static final String DIRECTION_MAP_CONTAINER_TAG = "direction";
    private static final String DIRECTION_MAP_FROM_TO_TAG = "from_to";
    private static final String DIRECTION_MAP_POLICY_TARGET_TAG = "vehicle";
    private static final String WITH_TRAFFIC_TURN_FOR_LANES_ON_RED_TAG = "allow_with_traffic_turn_on_red";
    private static final String ROAD_SPEC_TAG = "road";
    private static final String DELIM = "\\s*,\\s*";
    private static final String REG_EXPR_FOR_POLICY_TUPLES = "\\(|\\)\\s*,\\s*\\(|\\)";//should be used with String.split() to split tuples apart, leaving their contents in seperate strings
    //todo, this really should be moved somewhere else, this was a quick fix for an experiment
    //refactor so this is a value in the designated lanes expr that gets passed into grid map, and then the comparison is done when assigning values to lanes/roads
    private static final double MAX_RESERVATION_DISTANCE = Double.POSITIVE_INFINITY; //in meters, used when constructing roads with max reservation times. Multiplying the provided time times the speed will get the distance, which is capped to this value. This value is also used to specify time by dividing the speed limit if not set 

    public static ArchIntersection getIntersectionArchitectureFromXMLFile(File archFile) {
        try {
            //open XML and read root tag
            DocumentBuilderFactory dbfact = DocumentBuilderFactory.newInstance();
            DocumentBuilder dbuild = dbfact.newDocumentBuilder();
            Document doc = dbuild.parse(archFile);
            doc.getDocumentElement().normalize();

            Element root = doc.getDocumentElement(); //only expects 1 map per file
            return readArch(root); //actually do processing of XML
        } catch (ParserConfigurationException ex) {
            Logger.getLogger(ArchIntersectionFactory.class.getName()).log(Level.SEVERE, null, ex);
        } catch (IOException ex) {
            Logger.getLogger(ArchIntersectionFactory.class.getName()).log(Level.SEVERE, null, ex);
        } catch (org.xml.sax.SAXException ex) {
            Logger.getLogger(ArchIntersectionFactory.class.getName()).log(Level.SEVERE, null, ex);
        }
        return null;
    }

    private static ArchIntersection readArch(Element rootelem) {

        NodeList roadNodeList = rootelem.getElementsByTagName(ROAD_SPEC_TAG);
        EnumMap<Constants.Direction, Double> roadDirsMasterAndSpeeds = new EnumMap<Constants.Direction, Double>(Constants.Direction.class);
        //in (#incoming lanes, #outgoing lanes) format for the integers
        EnumMap<Constants.Direction, Integer[]> laneCounts = new EnumMap<Constants.Direction, Integer[]>(Constants.Direction.class);
        //max time ahead a vehicle can make a reservation for roads for AVs, may be overidden by lane specific value (which is not implemented as of 11/13/19).
        EnumMap<Constants.Direction, Double> roadLevelDefaultAheadReservationTime = new EnumMap<Constants.Direction, Double>(Constants.Direction.class);

        for (int i = 0; i < roadNodeList.getLength(); i++) {
            parseAndAddRoadDetails(roadNodeList.item(i), roadDirsMasterAndSpeeds, laneCounts, roadLevelDefaultAheadReservationTime);
        }

        EnumMap<Constants.Direction, ArrayList<ArchLaneSpec>> laneSpecs = new EnumMap<Constants.Direction, ArrayList<ArchLaneSpec>>(Constants.Direction.class);
        NodeList laneMapNodeList = rootelem.getElementsByTagName(DIRECTION_MAP_CONTAINER_TAG);
        for (int i = 0; i < laneMapNodeList.getLength(); i++) {
            parseAndAddArchLaneSpec(laneMapNodeList.item(i), laneCounts, laneSpecs);
        }

        //this is a might because this behavior is also dependent on the with traffic turn direction being allowed in the lane (that's a right turn in the United States)
        EnumMap<Constants.Direction, Set<Integer>> lanesThatMightSupportWithTrafficTurnOnRed = new EnumMap<Constants.Direction, Set<Integer>>(Constants.Direction.class);
        NodeList lanesWhichMightSupportTurnOnRed = rootelem.getElementsByTagName(WITH_TRAFFIC_TURN_FOR_LANES_ON_RED_TAG);
        if (lanesWhichMightSupportTurnOnRed.getLength() == 1) {
            SimConfig.useExplicitMappingsForWithTrafficTurnOnRed = true;
            parseAndAddWithTrafficTurnsOnRedAllowed(lanesWhichMightSupportTurnOnRed.item(0), lanesThatMightSupportWithTrafficTurnOnRed);
        } else if (lanesWhichMightSupportTurnOnRed.getLength() > 1) {
            throw new RuntimeException("Found multiple entries though only one is allowed in intersection architecture for tag: " + WITH_TRAFFIC_TURN_FOR_LANES_ON_RED_TAG);
        } //skip if tag isn't present

        //assemble roads
        EnumMap<Constants.Direction, ArchRoadSpec> roadSpecs = new EnumMap<Constants.Direction, ArchRoadSpec>(Constants.Direction.class);
        for (Constants.Direction roadDir : roadDirsMasterAndSpeeds.keySet()) {
            roadSpecs.put(roadDir, new ArchRoadSpec(roadDir, roadDirsMasterAndSpeeds.get(roadDir), laneCounts.get(roadDir)[0], laneCounts.get(roadDir)[1], laneSpecs.get(roadDir), roadLevelDefaultAheadReservationTime.get(roadDir), lanesThatMightSupportWithTrafficTurnOnRed.get(roadDir)));
        }

        return new ArchIntersection(roadSpecs);
    }

    private static void parseAndAddWithTrafficTurnsOnRedAllowed(Node infoForWithTrafficTurnOnRed, EnumMap<Constants.Direction, Set<Integer>> map) {
        String[] pairs = infoForWithTrafficTurnOnRed.getTextContent().split(REG_EXPR_FOR_POLICY_TUPLES);

        for (String turnOnRedAllowedPair : pairs) {
            if (turnOnRedAllowedPair.isEmpty()) {
                continue;
            }

            String[] turnStrings = turnOnRedAllowedPair.split(DELIM);
            if (turnStrings.length != 2) {
                throw new RuntimeException("Error, found syntactically invalid pair for turning on red, unable to parse: " + infoForWithTrafficTurnOnRed.getTextContent());
            }
           
            Constants.Direction td = Util.getParseEnumFromStringCaseInsen(turnStrings[0], Constants.Direction.class);
            int relativeLaneIndex =  Integer.parseInt(turnStrings[1]);
                    
            if (!map.containsKey(td)) {
                map.put(td, new HashSet<Integer>());
            }
            
            map.get(td).add(relativeLaneIndex);
        }
    }

    private static void parseAndAddArchLaneSpec(Node turningAndRestrictionInfo, EnumMap<Constants.Direction, Integer[]> validRoadsAndLaneCounts, EnumMap<Constants.Direction, ArrayList<ArchLaneSpec>> laneSpecs) {
        NodeList fromTo = ((Element) turningAndRestrictionInfo).getElementsByTagName(DIRECTION_MAP_FROM_TO_TAG);
        //check valid number of from_to tags
        if (fromTo.getLength() > 1) {
            throw new RuntimeException("Multiple from/to directions found in mapping when parsing file architecture and policy file. Please review parameters inside of <direction> tags and make sure that exactly 1 <from_to> set of tags exists within each <direction> tag pair.");
        } else if (fromTo.getLength() == 0) {
            throw new RuntimeException("No from/to tags found in direction mapping when parsing file architecture and policy file. Please review parameters inside of <direction> tags and make sure that exactly 1 <from_to> set of tags exists within each <direction> tag pair.");
        }

        String[] directions = fromTo.item(0).getTextContent().split(DELIM);
        //make sure split happened appropriately, there should only be 2 elements
        if (directions.length > 2) {
            throw new RuntimeException("Unable to parse direction mapping directions, unexpected number of elements: " + fromTo.item(0).getTextContent() + ". Expected 2, got: " + directions.length);
        }
        //store from & to directions
        Constants.Direction from = Util.getParseEnumFromStringCaseInsen(directions[0], Constants.Direction.class);
        Constants.Direction to = Util.getParseEnumFromStringCaseInsen(directions[1], Constants.Direction.class);
        if (!validRoadsAndLaneCounts.containsKey(from)) {
            String allowedDirs = "(";
            for (Constants.Direction allowedDir : Constants.Direction.values()) {
                allowedDirs += " " + allowedDir;
            }
            allowedDirs += ")";
            throw new RuntimeException("Invalid from direction specified in turning policy mapping: " + fromTo.item(0).getTextContent() + ". Make sure that details for the road are specified and match allowed directions. Allowed directions: " + allowedDirs);
        }

        //finish reading of policies by setting up to find appropriate ArchLaneSpec
        ArrayList<ArchLaneSpec> candidateLaneSpecs;
        if (laneSpecs.containsKey(from)) {
            candidateLaneSpecs = laneSpecs.get(from);
        } else {
            candidateLaneSpecs = new ArrayList<ArchLaneSpec>();
            laneSpecs.put(from, candidateLaneSpecs);
        }

        //parse the turn policy information
        NodeList policies = ((Element) turningAndRestrictionInfo).getElementsByTagName(DIRECTION_MAP_POLICY_TARGET_TAG);
        for (int i = 0; i < policies.getLength(); i++) {
            Node currentPolicy = policies.item(i);
            //get the type of vehicle the policy applies to
            SimConfig.VEHICLE_TYPE vType = Util.getParseEnumFromStringCaseInsen(currentPolicy.getAttributes().getNamedItem("type").getNodeValue(), SimConfig.VEHICLE_TYPE.class);
            String[] fileATurns = currentPolicy.getTextContent().split(REG_EXPR_FOR_POLICY_TUPLES);
            for (String fileAllowedTurn : fileATurns) {
                if (fileAllowedTurn.isEmpty()) {
                    continue;
                }

                String[] turnStrings = fileAllowedTurn.split(DELIM);
                if (turnStrings.length != 2) {
                    throw new RuntimeException("Error, found syntactically invalid turn policy, unable to parse: " + currentPolicy.getTextContent());
                }
                AllowedTurn turn = new AllowedTurn(Integer.parseInt(turnStrings[0]), Integer.parseInt(turnStrings[1]));
                checkValidTuple(from, to, turn, validRoadsAndLaneCounts); //throws exception if invalid
                ArchLaneSpec laneSpec;
                if (candidateLaneSpecs.size() > turn.getFrom()) {
                    laneSpec = candidateLaneSpecs.get(turn.getFrom());
                } else {
                    //make room for the lane if it doesn't yet exist
                    candidateLaneSpecs.ensureCapacity(turn.getFrom());
                    for (int j = candidateLaneSpecs.size(); j <= turn.getFrom(); j++) {
                        candidateLaneSpecs.add(null);
                    }
                    laneSpec = candidateLaneSpecs.get(turn.getFrom());
                }

                if (laneSpec == null) {
                    candidateLaneSpecs.set(turn.getFrom(), new ArchLaneSpec());
                    laneSpec = candidateLaneSpecs.get(turn.getFrom());
                }
                laneSpec.addPolicyAndRestriction(to, vType, turn);

            }
        }
    }

    private static void parseAndAddRoadDetails(Node roadDetails, EnumMap<Constants.Direction, Double> speeds, EnumMap<Constants.Direction, Integer[]> lanes, EnumMap<Constants.Direction, Double> roadLevelDefaultAheadReservationTime) {
        String[] details = roadDetails.getTextContent().split(DELIM);
        //expected number of parameters per road is 4
        if (details.length < 4) {
            throw new RuntimeException("Error parsing road details, found " + details.length + " expected 4. Road detail received: " + roadDetails.getTextContent());
        }
        Constants.Direction roadDir = Util.getParseEnumFromStringCaseInsen(details[0], Constants.Direction.class);

        //speeds is used to verify no duplicate directions for both maps
        if (speeds.containsKey(roadDir)) {
            throw new RuntimeException("Error parsing road details, duplicate road specified: " + roadDir.name());
        }

        int inLaneCount = Integer.parseInt(details[1]);
        int outLaneCount = Integer.parseInt(details[2]);
        double speed = Double.parseDouble(details[3]);
        double maxReservationTimeInSeconds;

        if (speed > 0) {
            speeds.put(roadDir, speed); //set the road speed
        } else {
            throw new RuntimeException("Invalid speed set for road (" + speed + "), speed must be strictly positive. Line: " + roadDetails.getTextContent());
        }
        if (inLaneCount > 0 && outLaneCount > 0) {
            lanes.put(roadDir, new Integer[]{inLaneCount, outLaneCount}); //set the [# of incoming lanes on road, #outgoing lanes on road]
        } else {
            throw new RuntimeException("Invalid number of lanes specified for road, both the incoming and outgoing number of lanes must be strictly positive. Line: " + roadDetails.getTextContent());
        }

        if (details.length >= 5) {
            maxReservationTimeInSeconds = Double.parseDouble(details[4]);
            if (maxReservationTimeInSeconds > 0) {
                //if (maxReservationTimeInSeconds*speed <= MAX_RESERVATION_DISTANCE) {
                roadLevelDefaultAheadReservationTime.put(roadDir, maxReservationTimeInSeconds);
                /*} else {
                    throw new RuntimeException("Invalid ahead reservation time set for road (" + maxReservationTimeInSeconds + "), Multiplying by speed of " + speed + " exceeds hard coded maximum allowed distance for reservations of " + MAX_RESERVATION_DISTANCE + ". Line: " + roadDetails.getTextContent());
                }*/
            } else {
                throw new RuntimeException("Invalid ahead reservation time set for road (" + maxReservationTimeInSeconds + "), ahead reservation time must be strictly positive. Line: " + roadDetails.getTextContent());
            }
        } else {
            roadLevelDefaultAheadReservationTime.put(roadDir, MAX_RESERVATION_DISTANCE / speed);
        }
    }

    //checks if from is outside allowed lane range or if to is outside allowed lane range on AllowedTurn for a lane
    private static void checkValidTuple(Constants.Direction from, Constants.Direction to, AllowedTurn tuple, EnumMap<Constants.Direction, Integer[]> validRoadsAndLaneCounts) {
        if (validRoadsAndLaneCounts.containsKey(from) && validRoadsAndLaneCounts.containsKey(to)) {
            //in (#incoming lanes, #outgoing lanes) format
            if (validRoadsAndLaneCounts.get(from) != null && validRoadsAndLaneCounts.get(to) != null) {
                if (tuple.getFrom() >= 0 && tuple.getFrom() < validRoadsAndLaneCounts.get(from)[0] && tuple.getTo() >= 0 && tuple.getTo() < validRoadsAndLaneCounts.get(to)[1]) {
                    return;
                } else {
                    throw new RuntimeException("Tuple with invalid lane parameters found: " + tuple.getStringRepresentation() + ". For these roads (In: " + from.name() + ", Out: " + to.name() + ") with the current settings, "
                            + "indices must be within the following integer ranges. Incoming: [0, " + (validRoadsAndLaneCounts.get(from)[0] - 1) + "] and outgoing: [0, " + (validRoadsAndLaneCounts.get(to)[1] - 1) + "]");
                }
            }
            //fall through
        }
        throw new RuntimeException("Invalid roads for tuple: " + tuple.getStringRepresentation() + ". In: " + from.name() + " Out:" + to.name() + ".");
    }

}
