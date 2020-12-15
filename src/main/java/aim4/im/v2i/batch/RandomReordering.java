package aim4.im.v2i.batch;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import aim4.im.v2i.RequestHandler.BatchModeRequestHandler.IndexedProposal;
import aim4.util.Util;

public class RandomReordering extends RoadBasedReordering implements ReorderingStrategy {

	public RandomReordering(double processingInterval) {
		super(processingInterval);
	}

	protected List<IndexedProposal> reorderProposals(
      List<IndexedProposal> iProposals) {
		List<IndexedProposal> result = new LinkedList<IndexedProposal>(iProposals);
		Collections.shuffle(result, Util.RANDOM_NUM_GEN);
		return result;
	}
}
