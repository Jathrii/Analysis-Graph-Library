/*
import java.util.Collections;
import java.util.Comparator;
*/
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.Hashtable;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Vector;
import java.lang.Math;

class Pair {
	String vertexID;
	int cost;

	public Pair(String vertexID, int cost) {
		this.vertexID = vertexID;
		this.cost = cost;
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

				// remove all edges connected to the vertex
				for (Edge edge : edgeList) {
					if (edge.get_vertex1ID().toString().equals(strVertexUniqueID)
							|| edge.get_vertex2ID().toString().equals(strVertexUniqueID))
						removeEdge(edge.getUniqueID().toString());
				}

				// remove vertex's adjacency list entry
				adjacencyList.remove(strVertexUniqueID);

				// remove vertex from other vertices' adjacency lists
				for (LinkedList<Pair> vertexAdjList : adjacencyList.values()) {
					for (Pair pair : vertexAdjList) {
						if (pair.vertexID.equals(strVertexUniqueID))
							vertexAdjList.remove(pair);
					}
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

				// remove vertex2 from vertex1's adjacency list
				for (Pair pair : vertex1AdjList) {
					if (pair.vertexID.equals(vertex2ID))
						vertex1AdjList.remove(pair);
				}

				// remove vertex1 from vertex2's adjacency list
				for (Pair pair : vertex2AdjList) {
					if (pair.vertexID.equals(vertex1ID))
						vertex2AdjList.remove(pair);
				}
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
	// returns the edge connecting 2 vertices
	public Edge connectingEdge(Vertex startVertex, Vertex endVertex) {
		String startVertexID = startVertex.getUniqueID().toString();
		String endVertexID = endVertex.getUniqueID().toString();

		for (Edge edge : edgeList) {
			if ((edge.get_vertex1ID().toString().equals(startVertexID)
					&& edge.get_vertex2ID().toString().equals(endVertexID))
					|| (edge.get_vertex1ID().toString().equals(endVertexID)
							&& edge.get_vertex2ID().toString().equals(startVertexID)))
				return edge;
		}

		return null;
	}
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

	/*
	public Vector<Edge> sortVectorOfEdges(Vector<Edge> vector) {
		Collections.sort(vector, Comparator.comparing(Edge::getCost));
		return vector;
	}
	*/

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
		 * //for testing purposes for (PathSegment pathSegment : pathVectorResult) {
		 * System.out.println("Vertex : "+pathSegment.getVertex().getUniqueID()
		 * +", Edge Cost : "+pathSegment.getEdge().getCost()); }
		 */
		return pathVectorResult;
	}

	// finds the closest pair of vertices using divide and conquer
	// algorithm. Use X and Y attributes in each vertex. [30 pts]
	public Vertex[] closestPair() throws GraphException 
	{
		
		Vector<Vertex> tmp = this.vertices();
		
		Collections.sort(tmp, OrderByX);
		
		Vertex [] res = closestPairHelper(this.vertices());
		
		return res;

	}
	
	double dist(Vertex a, Vertex b)
	{
		return Math.sqrt(Math.pow(a.getX() - b.getX(),2) + Math.pow((a.getY() - b.getY()), 2));
	}
	
	Vertex[] closestPairHelper(Vector<Vertex> P)
	{
		Vertex[] res = new Vertex[2];
		if(P.size() <= 3)
		{
			res[0] = P.get(0);
			res[1] = P.get(1);
			for( int i = 0; i <P.size(); i++)
			{
				for(int j = i + 1; j< P.size(); j++)
				{
					if(dist(P.get(i), P.get(j))< dist(res[0], res[1]))
					{
						res[0] = P.get(i);
						res[1] = P.get(j);
					}
				}
			}
			return res;
		}
		Vector<Vertex> Left = new Vector<Vertex>();
		Vector<Vertex> Right = new Vector<Vertex>();
		for(int i = 0; i < P.size() / 2; i++)
		{
			Left.add(P.get(i));
		}
		
		for(int i = P.size() / 2 + 1; i < P.size(); i++)
		{
			Right.add(P.get(i));
		}
		
		Vertex[] minLeft = closestPairHelper(Left);
		Vertex[] minRight = closestPairHelper(Right);
		
		if(dist(minLeft[0],minLeft[1]) <= dist(minRight[0], minRight[1]))
		{
			res = minLeft;
		}
		else
		{
			res = minRight;
		}
		
		Vector<Vertex> tmp = new Vector<Vertex>();
		Vertex [] tmpres = new Vertex[2];
		for (int i = 0; i < P.size(); i++)
		{
			if(Math.abs(P.get(P.size()/2).getX() - P.get(i).getX()) < dist(res[0],res[1]));
				tmp.add(P.get(i));
		}
		
		tmpres = closestPairHelper1(tmp, res);
		if(dist(tmpres[0], tmpres[1]) < dist(res[0],res[1]))
			res = tmpres;
		
		return res;
	}
	
	Comparator<Vertex> OrderByY =  new Comparator<Vertex>() {
        public int compare(Vertex s1, Vertex s2) {
            return s1.getY() - s2.getY();
        }
    };
    
    Comparator<Vertex> OrderByX =  new Comparator<Vertex>() {
        public int compare(Vertex s1, Vertex s2) {
            return s1.getX() - s2.getX();
        }
    };
	
	Vertex[] closestPairHelper1(Vector<Vertex> P, Vertex[] min)
	{
		P.sort(OrderByY);
		
		Vertex [] min1 = new Vertex[2];
		min1[0] = min[0];
		min1[1] = min[1];
		// Pick all points one by one and try the next points till the difference 
	    // between y coordinates is smaller than d. 
	    // This loop is proven to run at most 6 times 
		
		for(int i = 0; i < P.size(); i++)
		{
			for(int j = i + 1; j < P.size() && (Math.abs(P.get(i).getY() - P.get(j).getY())) < dist(min1[0], min1[1]); j++)
			{
				if(dist(P.get(i), P.get(j)) < dist(min1[0], min1[1]))
				{
					min1[0] = P.get(i);
					min1[1] = P.get(j);
				}
			}
		}
		
		return min1;	
	}
	// finds a minimum spanning tree using kruskal greedy algorithm
	// and returns the path to achieve that. Use Edge._nEdgeCost
	// attribute in finding the min span tree [30 pts]
	public Vector<PathSegment> minSpanningTree() throws GraphException {
		return null;

	}

	// finds shortest paths using bellman ford dynamic programming
	// algorithm and returns all such paths starting from given
	// vertex. Use Edge._nEdgeCost attribute in finding the
	// shortest path [35 pts]
	public Vector<Vector<PathSegment>> findShortestPathBF(String strStartVertexUniqueID) throws GraphException {
		return null;

	}

	// finds all shortest paths using Floyd�Warshall dynamic
	// programming algorithm and returns all such paths. Use
	// Edge._nEdgeCost attribute in finding the shortest path
	// [35 pts]
	public Vector<Vector<PathSegment>> findAllShortestPathsFW() throws GraphException {
		return null;

	}

}