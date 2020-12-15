H-AIM with Adaptive Signal Timing and Signal Actuation
==============

Code Base
==============
This is was taken from the AIM codebase authored by Tsz-Chiu Au, available here:
http://www.cs.utexas.edu/~aim/ under "The Hybrid-AIM Simulator" heading (last accessed 10/30/2020).

Further development was conducted by Aaron Parks-Young in coordination with Dr. Guni Sharon.

Running Experiments
==============
Experiments with the improvements made to the H-AIM simulator are designed for a single intersection with human and/or autonomous vehicles.

Running Experiments from the Command Line
--------------
In order to run an experiment without the GUI, compile the project (using Maven is suggested) with expr.trb.DesignatedLanesExpr as the main class. Run it as in:

java -jar AIM4-1.1a-SNAPSHOT.jar -1069075740 0 0 0 experiment.csv 0 0.1 0.3 ./exp/signal.xml ./exp/turnmovements.csv ./exp/intersection.xml false false

After the name of the .jar file ("AIM4-1.1a-SNAPSHOT.jar" if using the default build rules), the arguments in order are:
1. seed for random number generation
2. ratio of autonomous vehicles in the experiment \[0, 1\]+
3. ratio of cruise controlled vehicles in the experiment \[0, 1\]+\*
4. ratio of adaptive cruise controlled vehicles in the experiment \[0, 1\]+\*
5. full data output path including file name
6. scenario index for record keeping
7. safety buffer, time between reservations of a tile within the intersection^
8. exit tile safety buffer, time between reservations of a tile on the border of the intersection^
9. signal phase file path, this file dictates how traffic signals progress
10. turn movements file path, this file dictates how vehicles arrive and what turning movements they take
11. intersection (sometimes "architecture" or "arch") configuration file path, this file dictates how the intersection is constructed
12. flag for actuation (true to allow)
13. flag to use adaptive timing (true to allow)
 

Items with a + must sum to a number within the range \[0, 1\]. The human vehicle percentage is determined by subtracting this sum from 1 and is not range checked. Items with a \* have not been tested with the upgrades to the simulator, and so it's suggested to use these only after verifying they work with your use case. Items with a ^ might be best understood by reviewing A Multiagent Approach to Autonomous Intersection Management, Kurt Dresner and Peter Stone, The Journal of Artificial Intelligence Research (JAIR), March 2008 (sections 3.4 and 3.4.6). Note that for longer experimental runs, you may need to consider increasing the memory allocated to your JVM as in "-Xmx3500M".

Running the GUI
--------------
Simply running the application (e.g. "java -jar AIM4-1.1a-SNAPSHOT.jar") without arguments will cause the GUI to be started. The GUI assumes a 0.5% AV ratio, actuated timing, adaptive timing, and that the following files will be present:

1. exp/turnmovements.csv (describes how vehicles arrive and what turning movements they take)
2. exp/signal.xml (describes how traffic signals progress)
3. exp/intersection.xml (sometimes "architecture" or "arch", describes how the intersection is constructed)

Clicking in the intersection when the GUI is showing will bring up a window with information about the traffic signal configuration and history. Note that for longer experimental runs, you may need to consider increasing the memory allocated to your JVM as in "-Xmx3500M". Data is not saved for GUI-enabled runs of the simulator.

Customization of Simulation Input Files
==============
Sample input files have been provided in the exp folder. Below are instructions about how to generate your own files:

Signal Progression
--------------
exp/signal.xml shows how signal timing is specified in a ring and barrier fashion for the improved AIM Simulator. This configuration file is a standard XML file using <green>, <yellow>, and <red> tags to represent the various timings of phases within a pair of <ring> tags. The document may have multiple <ring> tag pairs specified. The <barrier> tag is both used to define a barrier and place it within a ring, depending on the position of the tag within the file. All tags should be contained within the <root> tags and the XML encoding and specification tag should be at the front of the document to enable correct parsing by the simulator.

Within each pair of <green> tags, there are first a series of 2 letters. The first letter represents the direction of travel for the road for which the signal applies (e.g., N for northbound, E for eastbound, etc.). The second letter represents the turning movement for which the signal should apply. Currently, the simulator only supports the explicit definition of turns which cross traffic (e.g., left in the United States) and through movements (which implies an allowed turn, such as right in the United States). The keys for each of those turning movements may be combined as in "tc" or "ct". <yellow> and <red> tag pairs following a pair of <green> tags should share the same direction and allowed turning movement parameters as the <green> tags they follow. Also note that a <green> tag's turning movement may have a "\^{}" or "*" included. Phases whose <green> tags have a "*" will be extended up until the end of another green signal in the same index (but a different ring) whose turning movement contains a "\^{}". This acts as a sort of "soft-barrier", should be used sparingly, and should be tested with specific signal configurations if used to ensure expected behavior.

Each pair of <green> tags also has 3 numbers contained within. The first number represents the amount of time in seconds to extend a green signal when a vehicle activates a sensor, assuming such extensions are enabled on the command line. The next number represents the minimum time in seconds the signal must remain green. The final number represents the maximum time in seconds a signal is allowed to be green under normal extension conditions (i.e., no safety constraint such as waiting on another phase to cross a barrier causing an artificial extension). The <yellow> and <red> tag pairs both function similarly to the <green> tag pairs, but require that only the duration be specified for the respectively yellow or red signal. Note that the simulator functions on discrete time steps of 0.02 seconds by default, and so updates to signals may only occur every 0.02 seconds. Each <ring> tag begins with a green signal and signals in the ring progress through yellow and red before returning to green again.

Barriers are identified by the <barrier> tag with a specific id attribute. When included within a ring, on top of enforcing phase progression constraints as the name "barrier" would suggest, this tag functions as both a <yellow> and <red> tag with timing based off of the specific barrier's definition. A barrier may be defined by including the <barrier> tag at the same level in the document as the <ring> tags. Each barrier (differentiated by the id attribute in the tag) should be referenced in each ring at appropriate points in the phase progression.
 
Turn Movements/Scheduling
--------------
exp/turnmovements.csv shows how the improved AIM Simulator ingests information about traffic arrivals in a comma delimited format (CSV). The first row dictates the order of directions of travel which spawned vehicles are assigned. The next row dictates the order of turning actions to be parsed for each road based on the first row. Thus, in exp/turnmovements.csv, the first series of "L,T,R" represents the left, through, and right turning directions for vehicles which spawn heading east in the simulator. The next series of "L,T,R" is for westbound vehicles, and so on. Note that if a compound action is encountered, such as "TR", the simulator will randomly split vehicles spawning amongst the composing turning actions using a uniform distribution. "Total" acts as a delimiter between travel directions on the second line and the associated value in each line of data is ignored. The "Vehicle Total" values are ignored as well. The rest of the file is comprised of multiple rows of arrival data.

Each row of arrival data consists of a timestamp followed by a series of numbers. The difference between timestamps on each line is used by the simulator to determine the span of time in which it may spawn the vehicles described for each line. The simulator requires that the difference in time between each line of data is the same and that at least 2 rows of data exist so that the time span may be determined automatically. Note that it is not currently recommended to use data which spans across multiple days, as testing of the simulator's parsing capability in this situation has not been performed. The numbers following the timestamp on each line correspond to the series of turn movements for each road on the second line of the file. Thus, the sequence "1,1,1" on the first line of data represents that 3 vehicles heading east should spawn in the first 5 simulation minutes with one choosing to turn left, one choosing to proceed straight, and one choosing to turn right. No spaces should occur on data lines, except for within the timestamp as shown.

Intersection Architecture
--------------
exp/intersection.xml shows an example file defining the allowed turning actions per lane, the lanes entering per road, the lanes exiting per road, speed limits per road in meters per second, and the maximum future time allowed for reservations per road in seconds. The file is an XML file, and so the XML encoding and specification tag should be at the front of the document to enable correct parsing by the simulator. The <intersection> tag pair following that tag is the root level tag. Within the <intersection> tag pair are <road> tags and the <direction> tags along with their sub-tags.

Each <road> tag contains 5 items. First is the direction of travel for a road. The directions chosen should be the same cardinal directions as in exp/turnmovements.csv. Second is the number of incoming lanes for the direction of travel followed by the number of outgoing lanes. Fourth is the speed in meters per second. Lastly is the maximum future time in seconds for which a reservation may be acquired for vehicles arriving on that road.

Next is a series of <direction> tag pairs and the sub-tags contained within. Each pair of <direction> tags defines a turning action and from which lanes the turning action may be taken for a particular road. The first sub-tag is the <from\_to> tag which dictates the cardinal direction from which a vehicle is traveling and to which the vehicle will be departing in order for the simulator to determine what turning action is being taken (left, right, straight). The from direction comes first and then is followed the departure direction. These directions are split by a comma. The next sub-tags are the <vehicle> sub-tags which contain a "type" parameter. The <vehicle> tags define which lanes for each vehicle type may be used to perform the turning action defined by the <from\_to> tag. The type parameter dictates the vehicle type and the pairs of numbers contained within the <vehicle> tag pair define the mapping of lanes. The lane mapping defines which incoming lanes a vehicle may use to enter the intersection before departing on particular outgoing lanes. This is currently a 1-to-1 mapping (per incoming road) with the first number being the relative index of the lane from the left on the incoming road (so, 0 would be the leftmost lane, 1 is the next lane to the right, etc.) and the second number being the relative index from the left on the outgoing road. 

Putting these all together, it can be observed that the first <direction> tag in exp/intersection.xml defines that all autonomous and human vehicles heading straight on the eastbound road must use the second lane from the left to proceed straight onto the leftmost lane on the outbound side of the intersection.