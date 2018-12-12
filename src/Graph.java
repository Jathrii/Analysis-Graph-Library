
/*
import java.util.Collections;
import java.util.Comparator;
*/
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.Hashtable;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Vector;
import java.lang.Math;

class Tuple {
	int x;
	int y;

	public Tuple(int x, int y) {
		this.x = x;
		this.y = y;
	}

	public int getX() {
		return x;
	}

	public int getY() {
		return y;
	}

	public String toString() {
		return "x : " + x + ", y : " + y + "\n";
	}
}

public class Graph {
	// LinkedList<Vertex> vertices = new LinkedList<Vertex>();
	Hashtable<String, Vertex> vertices = new Hashtable<String, Vertex>();
	LinkedList<Edge> edgeList = new LinkedList<Edge>();
	Hashtable<String, LinkedList<Pair>> adjacencyList = new Hashtable<String, LinkedList<Pair>>();

	// returns the name you have given to this graph library [1 pt]
	public String getLibraryName() {
		return "Graph";
	}

	// returns the current version number [1 pt]
	public String getLibraryVersion() {
		return "Version 1.01";
	}

	// the following method adds a vertex to the graph [2 pts]
	public void insertVertex(String strUniqueID, String strData, int nX, int nY) throws GraphException {
		boolean vertexExists = false;

		for (Vertex vertex : vertices.values()) {
			if (vertex.getUniqueID().toString().equals(strUniqueID))
				vertexExists = true;
		}

		if (!vertexExists)
			vertices.put(strUniqueID, new Vertex(new StringBuffer(strUniqueID), new StringBuffer(strUniqueID), nX, nY));
		else
			throw new GraphException("Vertex already exists");

	}

	// inserts an edge between 2 specified vertices [2 pts]
	public void insertEdge(String strVertex1UniqueID, String strVertex2UniqueID, String strEdgeUniqueID,
			String strEdgeData, int nEdgeCost) throws GraphException {
		boolean edgeExists = false;
		boolean vertex1Exists = false;
		boolean vertex2Exists = false;

		for (Edge edge : edgeList) {
			if (edge.getUniqueID().toString().equals(strEdgeUniqueID))
				edgeExists = true;
		}

		if (edgeExists)
			throw new GraphException("Edges already exists");

		for (Vertex vertex : vertices.values()) {
			if (vertex.getUniqueID().toString().equals(strVertex1UniqueID))
				vertex1Exists = true;
			if (vertex.getUniqueID().toString().equals(strVertex2UniqueID))
				vertex2Exists = true;
		}

		if (vertex1Exists && vertex2Exists) {
			// insert edge into edge list
			edgeList.add(new Edge(new StringBuffer(strVertex1UniqueID), new StringBuffer(strVertex2UniqueID),
					new StringBuffer(strEdgeUniqueID), new StringBuffer(strEdgeData), nEdgeCost));

			if (adjacencyList.containsKey(strVertex1UniqueID)) {
				adjacencyList.get(strVertex1UniqueID).add(new Pair(strVertex2UniqueID, nEdgeCost));
			} else {
				adjacencyList.put(strVertex1UniqueID, new LinkedList<Pair>());
				adjacencyList.get(strVertex1UniqueID).add(new Pair(strVertex2UniqueID, nEdgeCost));
			}
			if (adjacencyList.containsKey(strVertex2UniqueID)) {
				adjacencyList.get(strVertex2UniqueID).add(new Pair(strVertex1UniqueID, nEdgeCost));
			} else {
				adjacencyList.put(strVertex2UniqueID, new LinkedList<Pair>());
				adjacencyList.get(strVertex2UniqueID).add(new Pair(strVertex1UniqueID, nEdgeCost));
			}
		} else if (!(vertex1Exists || vertex2Exists)) {
			throw new GraphException("Both vertices don't exist");
		} else if (!vertex1Exists) {
			throw new GraphException("Vertex 1 doesn't exist");
		} else {
			throw new GraphException("Vertex 2 doesn't exist");
		}
	}

	// removes vertex and its incident edges [1 pt]
	public void removeVertex(String strVertexUniqueID) throws GraphException {
		for (Vertex vertex : vertices.values()) {
			if (vertex.getUniqueID().toString().equals(strVertexUniqueID)) {
				// remove vertex from the list of vertices
				vertices.remove(strVertexUniqueID);

				LinkedList<Edge> removedEdges = new LinkedList<Edge>();

				// remove all edges connected to the vertex
				for (Edge edge : edgeList) {
					if (edge.get_vertex1ID().toString().equals(strVertexUniqueID)
							|| edge.get_vertex2ID().toString().equals(strVertexUniqueID))
						removedEdges.add(edge);
				}

				for (Edge edge : removedEdges)
					removeEdge(edge.getUniqueID().toString());

				// remove vertex's adjacency list entry
				adjacencyList.remove(strVertexUniqueID);

				// remove vertex from other vertices' adjacency lists
				for (LinkedList<Pair> vertexAdjList : adjacencyList.values()) {
					LinkedList<Pair> removedPairs = new LinkedList<Pair>();

					for (Pair pair : vertexAdjList) {
						if (pair.vertexID.equals(strVertexUniqueID))
							removedPairs.add(pair);
					}

					for (Pair pair : removedPairs)
						vertexAdjList.remove(pair);
				}

				return;
			}
		}

		throw new GraphException("Vertex doesn't exist");
	}

	// removes an edge from the graph [1 pt]
	public void removeEdge(String strEdgeUniqueID) throws GraphException {
		for (Edge edge : edgeList) {
			if (edge.getUniqueID().toString().equals(strEdgeUniqueID)) {
				// remove edge from the list of edges
				edgeList.remove(edge);

				String vertex1ID = edge.get_vertex1ID().toString();
				String vertex2ID = edge.get_vertex2ID().toString();
				LinkedList<Pair> vertex1AdjList = adjacencyList.get(vertex1ID);
				LinkedList<Pair> vertex2AdjList = adjacencyList.get(vertex2ID);

				LinkedList<Pair> removed = new LinkedList<Pair>();

				// remove vertex2 from vertex1's adjacency list
				for (Pair pair : vertex1AdjList) {
					if (pair.vertexID.equals(vertex2ID))
						removed.add(pair);
				}

				for (Pair pair : removed)
					vertex1AdjList.remove(pair);

				removed = new LinkedList<Pair>();

				// remove vertex1 from vertex2's adjacency list
				for (Pair pair : vertex2AdjList) {
					if (pair.vertexID.equals(vertex1ID))
						removed.add(pair);
				}

				for (Pair pair : removed)
					vertex2AdjList.remove(pair);
			}

			return;
		}

		throw new GraphException("Edge doesn't exist");

	}

	// returns a vector of edges incident to vertex whose
	// id is strVertexUniqueID [1 pt]
	public Vector<Edge> incidentEdges(String strVertexUniqueID) throws GraphException {
		Vector<Edge> incidentEdgesVectors = new Vector<Edge>();
		for (Edge edge : edgeList) {
			if (edge.get_vertex1ID().toString().equals(strVertexUniqueID)
					|| edge.get_vertex2ID().toString().equals(strVertexUniqueID))
				incidentEdgesVectors.add(edge);
		}
		return incidentEdgesVectors;
	}

	// returns all vertices in the graph [1 pt]
	public Vector<Vertex> vertices() throws GraphException {
		return new Vector<Vertex>(vertices.values());
	}

	// returns all edges in the graph [1 pt]
	public Vector<Edge> edges() throws GraphException {
		return new Vector<Edge>(edgeList);
	}

	// returns an array of the two end vertices of the
	// passed edge [1 pt]
	public Vertex[] endVertices(String strEdgeUniqueID) throws GraphException {
		Vertex[] endVerticesArray = new Vertex[2];
		String[] vertexIDs = new String[2];
		vertexIDs[0] = null;

		for (Edge edge : edgeList) {
			if (edge.getUniqueID().toString().equals(strEdgeUniqueID)) {
				vertexIDs[0] = edge.get_vertex1ID().toString();
				vertexIDs[1] = edge.get_vertex2ID().toString();
			}
		}

		if (vertexIDs[0] == null)
			throw new GraphException("Edge doesn't exist");

		for (Vertex vertex : vertices.values()) {
			if (vertex.getUniqueID().toString().equals(vertexIDs[0]))
				endVerticesArray[0] = vertex;

			if (vertex.getUniqueID().toString().equals(vertexIDs[1]))
				endVerticesArray[1] = vertex;

		}
		return endVerticesArray;
	}

	// returns the vertex opposite of another vertex [1 pt]
	public Vertex opposite(String strVertexUniqueID, String strEdgeUniqueID) throws GraphException {
		String edgeID = null;
		String vertexID = null;

		for (Edge edge : edgeList) {
			if (edge.getUniqueID().toString().equals(strEdgeUniqueID)) {
				edgeID = edge.getUniqueID().toString();
				if (edge.get_vertex1ID().toString().equals(strVertexUniqueID))
					vertexID = edge.get_vertex2ID().toString();
				else if (edge.get_vertex2ID().toString().equals(strVertexUniqueID))
					vertexID = edge.get_vertex1ID().toString();
				else
					throw new GraphException("Vertex given is not associated with this edge");

			}
		}

		if (edgeID == null)
			throw new GraphException("Edge doesn't exist");

		for (Vertex vertex : vertices.values()) {
			if (vertex.getUniqueID().toString().equals(vertexID))
				return vertex;
		}

		return null;
	}

	/*
	 * // returns the edge connecting 2 vertices public Edge
	 * connectingEdge(Vertex startVertex, Vertex endVertex) { String
	 * startVertexID = startVertex.getUniqueID().toString(); String endVertexID
	 * = endVertex.getUniqueID().toString();
	 * 
	 * for (Edge edge : edgeList) { if
	 * ((edge.get_vertex1ID().toString().equals(startVertexID) &&
	 * edge.get_vertex2ID().toString().equals(endVertexID)) ||
	 * (edge.get_vertex1ID().toString().equals(endVertexID) &&
	 * edge.get_vertex2ID().toString().equals(startVertexID))) return edge; }
	 * 
	 * return null; }
	 */

	// performs depth first search starting from passed vertex
	// visitor is called on each vertex and edge visited. [12 pts]
	public void dfs(String strStartVertexUniqueID, Visitor visitor) throws GraphException {
		Hashtable<String, Boolean> visited = new Hashtable<String, Boolean>();
		for (String key : vertices.keySet()) {
			visited.put(key, false);
		}
		dfsHelper(strStartVertexUniqueID, visitor, visited);
	}

	public void dfsHelper(String strStartVertexUniqueID, Visitor visitor, Hashtable<String, Boolean> visited)
			throws GraphException {
		if (visited.get(strStartVertexUniqueID))
			return;
		else {
			visitor.visit(vertices.get(strStartVertexUniqueID));
			visited.put(strStartVertexUniqueID, true);
			// System.out.print(strStartVertexUniqueID+"->");
			Vector<Edge> adjEdgesVector = incidentEdges(strStartVertexUniqueID);
			for (Edge edge : adjEdgesVector) {
				Vertex oppositeVertex = opposite(strStartVertexUniqueID, edge.getUniqueID().toString());
				if (!visited.get(oppositeVertex.getUniqueID().toString())) {
					dfsHelper(oppositeVertex.getUniqueID().toString(), visitor, visited);
				}
			}
		}
	}

	// performs breadth first search starting from passed vertex
	// visitor is called on each vertex and edge visited. [17 pts]
	public void bfs(String strStartVertexUniqueID, Visitor visitor) throws GraphException {
		Hashtable<String, Boolean> visited = new Hashtable<String, Boolean>();
		Queue<Vertex> queue = new LinkedList<Vertex>();

		for (String key : vertices.keySet()) {
			visited.put(key, false);
		}

		Vertex startVertex = vertices.get(strStartVertexUniqueID);

		visitor.visit(startVertex);
		visited.put(strStartVertexUniqueID, true);
		queue.add(startVertex);

		while (!queue.isEmpty()) {
			Vertex parent = queue.poll();

			for (Edge edge : incidentEdges(parent.getUniqueID().toString())) {
				Vertex child = null;

				if (edge.get_vertex1ID().toString().equals(parent.getUniqueID().toString()))
					child = vertices.get(edge.get_vertex2ID().toString());
				else if (edge.get_vertex2ID().toString().equals(parent.getUniqueID().toString()))
					child = vertices.get(edge.get_vertex1ID().toString());

				if (!visited.get(child.getUniqueID().toString())) {
					visitor.visit(edge);
					visitor.visit(child);
					visited.put(child.getUniqueID().toString(), true);
					queue.add(child);
				}
			}

		}
	}

	public Vector<PathSegment> pathDFSHelper(String strStartVertexUniqueID, String strEndVertexUniqueID,
			Vector<PathSegment> pathVector, Vector<PathSegment> pathVectorResult, Hashtable<String, Boolean> visited)
			throws GraphException {
		visited.put(strStartVertexUniqueID, true);
		if (strStartVertexUniqueID.equals(strEndVertexUniqueID)) {
			for (PathSegment pathSegment : pathVector) {
				pathVectorResult.add(pathSegment);
			}
		}
		Vector<Edge> adjEdgesVector = incidentEdges(strStartVertexUniqueID);
		for (Edge edge : adjEdgesVector) {
			Vertex oppositeVertex = opposite(strStartVertexUniqueID, edge.getUniqueID().toString());
			if (!visited.get(oppositeVertex.getUniqueID().toString())) {
				pathVector.add(new PathSegment(vertices.get(strStartVertexUniqueID), edge));
				pathDFSHelper(oppositeVertex.getUniqueID().toString(), strEndVertexUniqueID, pathVector,
						pathVectorResult, visited);
				pathVector.remove(pathVector.size() - 1);
			}
		}
		return pathVectorResult;
	}

	// returns a path between start vertex and end vertex
	// if exists using dfs. [18 pts]
	public Vector<PathSegment> pathDFS(String strStartVertexUniqueID, String strEndVertexUniqueID)
			throws GraphException {
		Hashtable<String, Boolean> visited = new Hashtable<String, Boolean>();
		for (String key : vertices.keySet()) {
			visited.put(key, false);
		}
		Vector<PathSegment> pathVectorTemp = new Vector<PathSegment>();
		Vector<PathSegment> pathVectorResult = new Vector<PathSegment>();
		pathVectorResult = pathDFSHelper(strStartVertexUniqueID, strEndVertexUniqueID, pathVectorTemp, pathVectorResult,
				visited);
		/*
		 * //for testing purposes for (PathSegment pathSegment :
		 * pathVectorResult) {
		 * System.out.println("Vertex : "+pathSegment.getVertex().getUniqueID()
		 * +", Edge Cost : "+pathSegment.getEdge().getCost()); }
		 */
		return pathVectorResult;
	}

	// finds the closest pair of vertices using divide and conquer
	// algorithm. Use X and Y attributes in each vertex. [30 pts]
	public Vertex[] closestPair() throws GraphException {

		Vector<Vertex> tmp = this.vertices();

		Collections.sort(tmp, OrderByX);

		Vertex[] res = closestPairHelper(this.vertices());

		return res;

	}

	double dist(Vertex a, Vertex b) {
		return Math.sqrt(Math.pow(a.getX() - b.getX(), 2) + Math.pow((a.getY() - b.getY()), 2));
	}

	Vertex[] closestPairHelper(Vector<Vertex> P) {
		Vertex[] res = new Vertex[2];
		if (P.size() <= 3) {
			res[0] = P.get(0);
			res[1] = P.get(1);
			for (int i = 0; i < P.size(); i++) {
				for (int j = i + 1; j < P.size(); j++) {
					if (dist(P.get(i), P.get(j)) < dist(res[0], res[1])) {
						res[0] = P.get(i);
						res[1] = P.get(j);
					}
				}
			}
			return res;
		}
		Vector<Vertex> Left = new Vector<Vertex>();
		Vector<Vertex> Right = new Vector<Vertex>();
		for (int i = 0; i < P.size() / 2; i++) {
			Left.add(P.get(i));
		}

		for (int i = P.size() / 2 + 1; i < P.size(); i++) {
			Right.add(P.get(i));
		}

		Vertex[] minLeft = closestPairHelper(Left);
		Vertex[] minRight = closestPairHelper(Right);

		if (dist(minLeft[0], minLeft[1]) <= dist(minRight[0], minRight[1])) {
			res = minLeft;
		} else {
			res = minRight;
		}

		Vector<Vertex> tmp = new Vector<Vertex>();
		Vertex[] tmpres = new Vertex[2];
		for (int i = 0; i < P.size(); i++) {
			if (Math.abs(P.get(P.size() / 2).getX() - P.get(i).getX()) < dist(res[0], res[1]))
				;
			tmp.add(P.get(i));
		}

		tmpres = closestPairHelper1(tmp, res);
		if (dist(tmpres[0], tmpres[1]) < dist(res[0], res[1]))
			res = tmpres;

		return res;
	}

	Comparator<Vertex> OrderByY = new Comparator<Vertex>() {
		public int compare(Vertex s1, Vertex s2) {
			return s1.getY() - s2.getY();
		}
	};

	Comparator<Vertex> OrderByX = new Comparator<Vertex>() {
		public int compare(Vertex s1, Vertex s2) {
			return s1.getX() - s2.getX();
		}
	};

	Comparator<Vertex> OrderByID = new Comparator<Vertex>() {
		public int compare(Vertex s1, Vertex s2) {
			return (s1.getUniqueID().toString()).compareTo(s2.getUniqueID().toString());
		}
	};

	Vertex[] closestPairHelper1(Vector<Vertex> P, Vertex[] min) {
		P.sort(OrderByY);

		Vertex[] min1 = new Vertex[2];
		min1[0] = min[0];
		min1[1] = min[1];
		// Pick all points one by one and try the next points till the
		// difference
		// between y coordinates is smaller than d.
		// This loop is proven to run at most 6 times

		for (int i = 0; i < P.size(); i++) {
			for (int j = i + 1; j < P.size()
					&& (Math.abs(P.get(i).getY() - P.get(j).getY())) < dist(min1[0], min1[1]); j++) {
				if (dist(P.get(i), P.get(j)) < dist(min1[0], min1[1])) {
					min1[0] = P.get(i);
					min1[1] = P.get(j);
				}
			}
		}

		return min1;
	}

	// detects cycles in a sub-graph using dfs
	public boolean detectCycle(String vertex, String parent, Hashtable<String, Boolean> visited,
			Hashtable<String, Boolean> includedEdges) throws GraphException {

		visited.put(vertex, true);

		Vector<Edge> incidentEdges = incidentEdges(vertex);

		for (Edge incidentEdge : incidentEdges) {
			if (!includedEdges.get(incidentEdge.getUniqueID().toString()))
				continue;

			String oppositeVertex = opposite(vertex, incidentEdge.getUniqueID().toString()).getUniqueID().toString();

			if (visited.get(oppositeVertex)) {
				if (!oppositeVertex.equals(parent))
					return true;
			} else if (detectCycle(oppositeVertex, vertex, visited, includedEdges))
				return true;
		}

		return false;
	}

	// sorts a given vector of edges
	public Vector<Edge> sortVectorOfEdges(Vector<Edge> vector) {
		Collections.sort(vector, Comparator.comparing(Edge::getCost));
		return vector;
	}

	// finds a minimum spanning tree using kruskal greedy algorithm
	// and returns the path to achieve that. Use Edge._nEdgeCost
	// attribute in finding the min span tree [30 pts]
	public Vector<PathSegment> minSpanningTree() throws GraphException {
		Vector<Edge> sortedEdges = sortVectorOfEdges(edges());
		Hashtable<String, Boolean> includedEdges = new Hashtable<String, Boolean>();
		Vector<PathSegment> mst = new Vector<PathSegment>();

		// initialize hash table with false for all edges
		for (Edge edge : sortedEdges)
			includedEdges.put(edge.getUniqueID().toString(), false);

		for (Edge edge : sortedEdges) {
			includedEdges.put(edge.getUniqueID().toString(), true);

			String theChosenVertex = edge.get_vertex1ID().toString();
			Hashtable<String, Boolean> visited = new Hashtable<String, Boolean>();

			for (Vertex vertex : vertices())
				visited.put(vertex.getUniqueID().toString(), false);

			if (detectCycle(theChosenVertex, theChosenVertex, visited, includedEdges))
				includedEdges.put(edge.getUniqueID().toString(), false);
			else
				mst.add(new PathSegment(null, edge));
		}

		return mst;
	}

	public Edge findEdge(Vertex v1, Vertex v2) throws GraphException {
		for (Edge edge : incidentEdges(v1.getUniqueID().toString())) {
			Vertex tmpVertex = opposite(v1.getUniqueID().toString(), edge.getUniqueID().toString());
			if (tmpVertex.getUniqueID().toString().equals(v2.getUniqueID().toString())) {
				return edge;
			}
		}
		return null;
	}

	// finds shortest paths using bellman ford dynamic programming
	// algorithm and returns all such paths starting from given
	// vertex. Use Edge._nEdgeCost attribute in finding the
	// shortest path [35 pts]
	public Vector<Vector<PathSegment>> findShortestPathBF(String strStartVertexUniqueID) throws GraphException {
		Vector<Vertex> sortedVertices = this.vertices();
		Vector<Vector<PathSegment>> res = new Vector<Vector<PathSegment>>();
		Collections.sort(sortedVertices, OrderByID);
		int N = sortedVertices.size();
		Vector<String> VertexID = new Vector<String>();
		int startVertexIndex = -1;
		for (int i = 0; i < N; i++) {
			if (sortedVertices.get(i).toString().equals(strStartVertexUniqueID)) {
				startVertexIndex = i;
			}
			VertexID.insertElementAt(sortedVertices.get(i).toString(), i);
		}
		Vector<Edge> edges = this.edges();
		int[] dist = new int[N];
		for (int i = 0; i < N; i++) {
			if (i != startVertexIndex) {
				dist[i] = Integer.MAX_VALUE;
			}
		}

		Edge[][] adjMatrix = new Edge[N][N];
		for (int i = 0; i < N; i++) {
			Vertex tmpVertex = sortedVertices.get(i);
			for (int j = 0; j < edges.size(); j++) {
				Edge tmpEdge = edges.get(j);
				if (tmpEdge.get_vertex1ID().toString().equals(tmpVertex.getUniqueID().toString())) {
					int tmp = VertexID.indexOf(tmpEdge.get_vertex2ID().toString());
					adjMatrix[i][tmp] = tmpEdge;
				}
			}
		}

		ArrayList<Tuple> vertexTuple = new ArrayList<Tuple>();
		for (int i = 0; i < N; i++) {
			for (int j = 0; j < edges.size(); j++) {
				Edge edgeCurrent = edges.get(j);
				String u = edgeCurrent.get_vertex1ID().toString();
				String v = edgeCurrent.get_vertex2ID().toString();
				int weight = edgeCurrent.getCost();
				int uIndex = VertexID.indexOf(u);
				int vIndex = VertexID.indexOf(v);
				if (dist[uIndex] != Integer.MAX_VALUE && dist[uIndex] + weight < dist[vIndex]) {
					dist[vIndex] = dist[uIndex] + weight;
					vertexTuple.add(new Tuple(uIndex, vIndex));
				} else {
					v = edgeCurrent.get_vertex1ID().toString();
					u = edgeCurrent.get_vertex2ID().toString();
					uIndex = VertexID.indexOf(u);
					vIndex = VertexID.indexOf(v);
					if (dist[uIndex] != Integer.MAX_VALUE && dist[uIndex] + weight < dist[vIndex]) {
						dist[vIndex] = dist[uIndex] + weight;
						vertexTuple.add(new Tuple(uIndex, vIndex));
					}
				}

			}
		}
		for (int i = 0; i < N; i++) {
			boolean occurred = false;
			for (int j = 0; j < vertexTuple.size(); j++) {
				Tuple tmpTuple = vertexTuple.get(vertexTuple.size() - 1 - j);
				if (occurred && tmpTuple.y == i) {
					vertexTuple.remove(tmpTuple);
				} else {
					if ((!occurred) && tmpTuple.y == i) {
						occurred = true;
					}
				}
			}
		}
		for (int j = 0; j < edges.size(); ++j) {
			Edge edgeCurrent = edges.get(j);
			String u = edgeCurrent.get_vertex1ID().toString();
			String v = edgeCurrent.get_vertex2ID().toString();
			int weight = edgeCurrent.getCost();
			int uIndex = VertexID.indexOf(u);
			int vIndex = VertexID.indexOf(v);
			if (dist[uIndex] != Integer.MAX_VALUE && dist[uIndex] + weight < dist[vIndex]) {
				System.out.println("Graph contains negative weight cycle");
				return null;
			}
		}
		ArrayList<ArrayList<Tuple>> pathsToSourceVertexMultipleTuples = new ArrayList<ArrayList<Tuple>>();
		ArrayList<Tuple> pathsToSourceVertexSingleTuples = new ArrayList<Tuple>();
		for (int i = 0; i < vertexTuple.size(); i++) {
			Tuple currentTuple = vertexTuple.get(i);
			if (currentTuple.getX() == startVertexIndex) {
				pathsToSourceVertexSingleTuples.add(currentTuple);
			} else {
				ArrayList<Tuple> currentPath = new ArrayList<Tuple>();
				Tuple adjTuple = vertexTuple.get(0);
				findAdj(currentTuple, adjTuple, startVertexIndex, 0, vertexTuple, currentPath);
				currentPath.add(currentTuple);
				while (currentPath.get(0).getX() != startVertexIndex) {
					currentTuple = currentPath.get(0);
					adjTuple = vertexTuple.get(0);
					ArrayList<Tuple> tmpPath = new ArrayList<Tuple>();
					findAdj(currentTuple, adjTuple, startVertexIndex, 0, vertexTuple, tmpPath);
					// tmpPath.remove(currentPath.size()-1);
					while (!currentPath.isEmpty()) {
						tmpPath.add(currentPath.remove(0));
					}
					while (!tmpPath.isEmpty()) {
						currentPath.add(tmpPath.remove(0));
					}
				}
				pathsToSourceVertexMultipleTuples.add(currentPath);
			}
		}

		Vector<Vector<Vertex>> allPathToSourceVertexVertciesVector = new Vector<Vector<Vertex>>();
		ArrayList<Edge> edgesList = new ArrayList<Edge>();
		for (int i = 0; i < pathsToSourceVertexSingleTuples.size(); i++) {
			Vector<Vertex> singleCurrentPathVertciesToSourceVertex = new Vector<Vertex>();
			Tuple currentTuple = pathsToSourceVertexSingleTuples.get(i);
			singleCurrentPathVertciesToSourceVertex.add(sortedVertices.get(currentTuple.getX()));
			singleCurrentPathVertciesToSourceVertex.add(sortedVertices.get(currentTuple.getY()));
			edgeList.add(findEdge(sortedVertices.get(currentTuple.getX()), sortedVertices.get(currentTuple.getY())));
			// edgesList.add(adjMatrix[currentTuple.getX()][currentTuple.getY()]);
			allPathToSourceVertexVertciesVector.add(singleCurrentPathVertciesToSourceVertex);
		}

		for (Edge edgy : edgesList) {
			System.out.print(edgy.getUniqueID().toString() + ",");
		}
		System.out.println();
		for (int i = 0; i < pathsToSourceVertexMultipleTuples.size(); i++) {
			Vector<Vertex> singleCurrentPathVertciesToSourceVertex = new Vector<Vertex>();
			ArrayList<Tuple> currentArray = pathsToSourceVertexMultipleTuples.get(i);
			for (int j = 0; j < currentArray.size(); j++) {
				Tuple currentTuple = currentArray.get(j);
				if (j == 0) {
					singleCurrentPathVertciesToSourceVertex.add(sortedVertices.get(currentTuple.getX()));
				}
				singleCurrentPathVertciesToSourceVertex.add(sortedVertices.get(currentTuple.getY()));
				edgeList.add(findEdge(sortedVertices.get(currentTuple.getX()), sortedVertices.get(currentTuple.getY())));
			}
			allPathToSourceVertexVertciesVector.add(singleCurrentPathVertciesToSourceVertex);
		}
//		for (Vector<Vertex> vecSeg : allPathToSourceVertexVertciesVector) {
//			for (Vertex vec : vecSeg) {
//				System.out.print(vec.getUniqueID().toString() + "==>");
//			}
//			System.out.println();
//		}
		//System.out.println("+++_------___");
		Vector<Vector<PathSegment>> pathSegments = new Vector<Vector<PathSegment>>();
		int edgeIndex = 0;
		//System.out.println(Arrays.toString(edgeList.toArray()));
		for (Vector<Vertex> vecSeg : allPathToSourceVertexVertciesVector) {
			Vector<PathSegment> pathSegmentsSegment = new Vector<PathSegment>();
			for (int i = 0; i < vecSeg.size(); i++) {
				// findEdge
				// PathSegment pathSegment = new PathSegment(vecSeg.get(i),
				// edgesList.get(edgeIndex));
				//System.out.println(findEdge(vecSeg.get(i), vecSeg.get(i + 1)).getUniqueID().toString());
				PathSegment pathSegment = new PathSegment(vecSeg.get(i), edgeList.get(edgeIndex));
				//System.out.print(pathSegment.getVertex().getUniqueID().toString()+"==>");
				edgeIndex++;
				pathSegmentsSegment.add(pathSegment);
			}
			System.out.println();
			pathSegments.add(pathSegmentsSegment);
		}

		return pathSegments;
	}

	ArrayList<Tuple> findAdj(Tuple tuple1, Tuple tuple2, int sourceVertex, int index, ArrayList<Tuple> tupleArray,
			ArrayList<Tuple> result) {
		if (index == tupleArray.size()) {
			return result;
		}
		if (tuple2.getY() == tuple1.getX() && tuple2.getX() == sourceVertex) {
			result.add(tuple2);
			return result;
		}
		if (tuple2.getY() == tuple1.getX()) {
			result.add(tuple2);
			return findAdj(tuple2, tupleArray.get(index++), sourceVertex, index, tupleArray, result);
		} else {
			return findAdj(tuple1, tupleArray.get(index++), sourceVertex, index, tupleArray, result);
		}
	}

	// finds all shortest paths using Floyd�Warshall dynamic
	// programming algorithm and returns all such paths. Use
	// Edge._nEdgeCost attribute in finding the shortest path
	// [35 pts]
	public Vector<Vector<PathSegment>> findAllShortestPathsFW() throws GraphException {
		Vector<Vertex> sortedvertices = this.vertices();
		Vector<Vector<PathSegment>> res = new Vector<Vector<PathSegment>>();
		Collections.sort(sortedvertices, OrderByID);
		int N = sortedvertices.size();
		Vector<String> VertexID = new Vector<String>();
		for (int i = 0; i < N; i++) {
			VertexID.insertElementAt(sortedvertices.get(i).toString(), i);
		}
		Vector<Edge> Edges = this.edges();
		int[][] cost = new int[N][N];
		int[][] path = new int[N][N];
		for (int i = 0; i < N; i++) {
			Vertex tmpVertex = sortedvertices.get(i);
			for (int j = 0; j < Edges.size(); j++) {
				Edge tmpEdge = Edges.get(j);
				if (tmpEdge.get_vertex1ID().toString().equals(tmpVertex.getUniqueID().toString())) {
					int tmp = VertexID.indexOf(tmpEdge.get_vertex2ID().toString());
					cost[i][tmp] = tmpEdge.getCost();
					cost[tmp][i] = tmpEdge.getCost();
				}
			}
		}

		for (int x = 0; x < N; x++) {
			for (int y = 0; y < N; y++) {
				if (cost[x][y] == 0 && x != y) {
					cost[x][y] = Integer.MAX_VALUE;
				}
			}
		}

		for (int v = 0; v < N; v++) {
			for (int u = 0; u < N; u++) {
				if (v == u)
					path[v][u] = 0;
				else if (cost[v][u] != Integer.MAX_VALUE)
					path[v][u] = u;
				else
					path[v][u] = -1;
			}
		}

		for (int k = 0; k < N; k++) {
			for (int v = 0; v < N; v++) {
				for (int u = 0; u < N; u++) {

					if (cost[v][k] != Integer.MAX_VALUE && cost[k][u] != Integer.MAX_VALUE
							&& (cost[v][k] + cost[k][u] < cost[v][u])) {
						cost[v][u] = cost[v][k] + cost[k][u];
						path[v][u] = path[v][k];

					}
				}

				if (cost[v][v] < 0) {
					System.out.println("Negative Weight Cycle Found!!");
					return null;
				}
			}
		}
		Vector<Vector<Vertex>> tmpPaths = getSolution(cost, path, N, sortedvertices);

		for (int i = 0; i < tmpPaths.size(); i++) {
			Vector<PathSegment> Path = new Vector<PathSegment>();
			for (int j = 0; j < tmpPaths.get(i).size() - 1; j++) {
				PathSegment PS = null;
				for (int k = 0; k < Edges.size(); k++) {
					String vertex1ID = Edges.get(k).get_vertex1ID().toString();
					String vertex2ID = Edges.get(k).get_vertex2ID().toString();
					String currentVertexID = tmpPaths.get(i).get(j).getUniqueID().toString();
					String nextVertexID = tmpPaths.get(i).get(j + 1).getUniqueID().toString();
					
					boolean condA = vertex1ID.equals(currentVertexID) && vertex2ID.equals(nextVertexID);
					boolean condB = vertex2ID.equals(currentVertexID) && vertex1ID.equals(nextVertexID);
					if (condA || condB) {
						PS = new PathSegment(tmpPaths.get(i).get(j), Edges.get(k));
						break;
					}
					else if (condB) {
						PS = new PathSegment(tmpPaths.get(i).get(j), Edges.get(k));
						break;
					}
				}
				Path.insertElementAt(PS, j);
			}
			res.insertElementAt(Path, i);
		}

		return res;
	}

	private static void followPath(int[][] path, int v, int u, Vector<Vertex> vertices, Vector<Vertex> tmpPath) {
		if (path[v][u] == u)
			return;
		
		tmpPath.add(vertices.get(path[v][u]));
		
		followPath(path, path[v][u], u, vertices, tmpPath);
		// System.out.print(vertices.get(path[v][u]).getUniqueID().toString() +
		// " ");
		
	}

	private static Vector<Vector<Vertex>> getSolution(int[][] cost, int[][] path, int N, Vector<Vertex> vertices) {
		Vector<Vector<Vertex>> tmpPaths = new Vector<Vector<Vertex>>();
		for (int v = 0; v < N; v++) {
			for (int u = 0; u < N; u++) {
				if (u != v && path[v][u] != -1) {
					Vector<Vertex> tmpPath = new Vector<Vertex>();
					tmpPath.add(vertices.get(v));
					// System.out.print("Shortest Path from vertex " +
					// vertices.get(v).getUniqueID().toString() +
					// " to vertex " + vertices.get(u).getUniqueID().toString()
					// + " is (" + vertices.get(v).getUniqueID().toString() + "
					// ");
					followPath(path, v, u, vertices, tmpPath);
					// System.out.println(vertices.get(u).getUniqueID().toString()
					// + ")");
					tmpPath.add(vertices.get(u));
					tmpPaths.add(tmpPath);
				}
			}
		}
		return tmpPaths;
	}

}