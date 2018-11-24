class GradingVisitor implements Visitor {
	protected String _strResult = new String();

	public void visit(Vertex v) {
		//_strResult += "v=" + v.getUniqueID() + " ";
		_strResult += v.getUniqueID();
	}

	public void visit(Edge e) {
		//_strResult += "e=" + e.getUniqueID() + " ";
		//_strResult += e.getUniqueID();

	}

	public String getResult() {
		return _strResult;
	}
}

public class Grading {
	public int runTestCase1() throws GraphException {
		int nMark = 0;
		Graph g = new Graph();
		GradingVisitor gVisitor = new GradingVisitor();

		// Graph 1
//		g.insertVertex("1", "1", 0, 0);
//		g.insertVertex("2", "2", 0, 0);
//		g.insertVertex("3", "3", 0, 0);
//		g.insertVertex("4", "4", 0, 0);
//		g.insertVertex("5", "5", 0, 0);
//		g.insertVertex("6", "6", 0, 0);
//		g.insertEdge("1", "4", "88", "88", 5);
//		g.insertEdge("1", "2", "2", "2", 2);
//		g.insertEdge("2", "3", "14", "14", 14);
//		g.insertEdge("2", "4", "99", "99", 5);
//		g.insertEdge("2", "5", "4", "4", 4);
//		g.insertEdge("4", "5", "58", "58", 58);
//		g.insertEdge("3", "5", "34", "34", 34);
		
		// Graph 6
		g.insertVertex("D", "D", 2, 5);
		g.insertVertex("B", "B", 4, 7);
		g.insertVertex("A", "A", 19, 5);
		g.insertVertex("C", "C", 9, 3);
		g.insertVertex("E", "E", 1, 17);
		g.insertVertex("F", "F", 3, 9);
		g.insertVertex("G", "G", 5, 12);
		g.insertEdge("D", "B", "11", "88", 5);
		g.insertEdge("B", "A", "40", "2", 2);
		g.insertEdge("B", "C", "30", "14", 14);
		g.insertEdge("A", "C", "20", "99", 5);
		g.insertEdge("C", "E", "15", "4", 4);
		g.insertEdge("E", "F", "44", "58", 58);
		g.insertEdge("E", "G", "90", "34", 34);

		System.out.println("BFS:");
		g.bfs("C", gVisitor);
		System.out.println(gVisitor.getResult());
		
		gVisitor = new GradingVisitor();
		
		System.out.println("DFS:");
		g.dfs("C", gVisitor);
		System.out.println(gVisitor.getResult());
		
		gVisitor = new GradingVisitor();
		
		System.out.println("pathDFS:");
		g.pathDFS("D", "F");
		System.out.println(gVisitor.getResult());
		
		Vertex [] ClosestPair = g.closestPair();
		
		System.out.println("ClosestPair = { " + ClosestPair[0].getUniqueID() + " , " + ClosestPair[1].getUniqueID() + " }");
		
		/*
		if (gVisitor.getResult().equalsIgnoreCase("blah"))
			nMark += 12;
		*/

		return nMark;
	}

	public static void main(String[] args) throws GraphException {
		int nTotalMark = 0;
		Grading grading = new Grading();
		nTotalMark += grading.runTestCase1();
		System.out.println(nTotalMark);
	}
}