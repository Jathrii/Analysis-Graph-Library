class GradingVisitor implements Visitor {
	protected String _strResult = new String();

	public void visit(Vertex v) {
		_strResult += "v=" + v.getUniqueID() + " ";
	}

	public void visit(Edge e) {
		_strResult += "e=" + e.getUniqueID() + " ";

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

		g.insertVertex(new StringBuffer("1"), new StringBuffer("1"));
		g.insertVertex(new StringBuffer("2"), new StringBuffer("2"));
		g.insertVertex(new StringBuffer("3"), new StringBuffer("3"));
		g.insertVertex(new StringBuffer("4"), new StringBuffer("4"));
		g.insertVertex(new StringBuffer("5"), new StringBuffer("5"));
		g.insertEdge(new StringBuffer("1"), new StringBuffer("4"), new StringBuffer("5"), new StringBuffer("5"), 5);
		g.insertEdge(new StringBuffer("1"), new StringBuffer("2"), new StringBuffer("2"), new StringBuffer("2"), 2);
		g.insertEdge(new StringBuffer("2"), new StringBuffer("3"), new StringBuffer("14"), new StringBuffer("14"), 14);
		g.insertEdge(new StringBuffer("2"), new StringBuffer("4"), new StringBuffer("15"), new StringBuffer("5"), 5);
		g.insertEdge(new StringBuffer("2"), new StringBuffer("5"), new StringBuffer("4"), new StringBuffer("4"), 4);
		g.insertEdge(new StringBuffer("4"), new StringBuffer("5"), new StringBuffer("58"), new StringBuffer("58"), 58);
		g.insertEdge(new StringBuffer("3"), new StringBuffer("5"), new StringBuffer("34"), new StringBuffer("34"), 34);

		g.bfs(new StringBuffer("1"), gVisitor);

		if (gVisitor.getResult().equalsIgnoreCase("blah"))
			nMark += 12;
		
		System.out.println(gVisitor.getResult());

		return nMark;
	}

	public static void main(String[] args) throws GraphException {
		int nTotalMark = 0;
		Grading grading = new Grading();
		nTotalMark += grading.runTestCase1();
		System.out.println(nTotalMark);
	}
}