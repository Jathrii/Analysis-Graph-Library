import java.util.Hashtable;
import java.util.LinkedList;
import java.util.Vector;

class Pair {
	StringBuffer vertexID;
	int cost;

	public Pair(StringBuffer vertexID, int cost) {
		this.vertexID = vertexID;
		this.cost = cost;
	}
}

public class Graph {
	LinkedList<Vertex> vertices = new LinkedList<Vertex>();
	LinkedList<Edge> edgeList = new LinkedList<Edge>();
	Hashtable<StringBuffer, LinkedList<Pair>> adjacencyList = new Hashtable<StringBuffer, LinkedList<Pair>>();

	// returns the name you have given to this graph library [1 pt]
	public String getLibraryName() {
		return "Graph";
	}

	// returns the current version number [1 pt]
	public String getLibraryVersion() {
		return "Version 1.01";
	}

	// the following method adds a vertex to the graph [2 pts]
	public void insertVertex(StringBuffer strUniqueID, StringBuffer strData) throws GraphException {
		boolean vertexExists = false;
		for (Vertex vertex : vertices) {
			if (vertex.getUniqueID().toString().equals(strUniqueID.toString()))
				vertexExists = true;
		}
		if (!vertexExists)
			vertices.add(new Vertex(strUniqueID, strUniqueID));
		else
			throw new GraphException("Vertex already exists");

	}

	// inserts an edge between 2 specified vertices [2 pts]
	public void insertEdge(StringBuffer strVertex1UniqueID, StringBuffer strVertex2UniqueID,
			StringBuffer strEdgeUniqueID, StringBuffer strEdgeData, int nEdgeCost) throws GraphException {
		boolean edgeExists = false;
		boolean vertex1Exists = false;
		boolean vertex2Exists = false;

		for (Edge edge : edgeList) {
			if (edge.getUniqueID().toString().equals(strEdgeUniqueID.toString().toString()))
				edgeExists = true;
		}

		if (edgeExists)
			throw new GraphException("Edges already exists");

		for (Vertex vertex : vertices) {
			if (vertex.getUniqueID().toString().equals(strVertex1UniqueID))
				vertex1Exists = true;
			if (vertex.getUniqueID().toString().equals(strVertex2UniqueID))
				vertex2Exists = true;
		}

		if (vertex1Exists && vertex2Exists) {
			edgeList.add(new Edge(strVertex1UniqueID, strVertex2UniqueID, strEdgeUniqueID, strEdgeData, nEdgeCost));
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
	@SuppressWarnings("unlikely-arg-type")
	public void removeVertex(StringBuffer strVertexUniqueID) throws GraphException {
		for (Vertex vertex : vertices) {
			if (vertex.getUniqueID().toString().equals(strVertexUniqueID.toString())) {
				// remove vertex from the list of vertices
				vertices.remove(strVertexUniqueID);

				// remove all edges connected to the vertex
				for (Edge edge : edgeList) {
					if (edge.get_vertex1ID().toString().equals(strVertexUniqueID.toString())
							|| edge.get_vertex2ID().toString().equals(strVertexUniqueID.toString()))
						removeEdge(edge.getUniqueID());
				}

				// remove vertex's adjacency list entry
				adjacencyList.remove(strVertexUniqueID);

				// remove vertex from other vertices' adjacency lists
				adjacencyList.forEach((key, value) -> {
					for (Pair pair : value) {
						if (pair.vertexID.toString().equals(strVertexUniqueID.toString()))
							value.remove(pair);
					}
				});

				return;
			}
		}

		throw new GraphException("Vertex doesn't exist");
	}

	// removes an edge from the graph [1 pt]
	@SuppressWarnings("unlikely-arg-type")
	public void removeEdge(StringBuffer strEdgeUniqueID) throws GraphException {
		for (Edge edge : edgeList) {
			if (edge.getUniqueID().toString().equals(strEdgeUniqueID.toString())) {
				// remove edge from the list of edges
				edgeList.remove(strEdgeUniqueID);
				
				String vertex1ID = edge.get_vertex1ID().toString();
				String vertex2ID = edge.get_vertex2ID().toString();
				LinkedList<Pair> vertex1AdjList = adjacencyList.get(edge.get_vertex1ID());
				LinkedList<Pair> vertex2AdjList = adjacencyList.get(edge.get_vertex2ID());
				
				// remove vertex2 from vertex1's adjacency list
				for (Pair pair: vertex1AdjList) {
					if (pair.vertexID.toString().equals(vertex2ID))
						vertex1AdjList.remove(pair);
				}
				
				// remove vertex1 from vertex2's adjacency list
				for (Pair pair: vertex2AdjList) {
					if (pair.vertexID.toString().equals(vertex1ID))
						vertex2AdjList.remove(pair);
				}
			}
			
			return;
		}
		
		throw new GraphException("Edge doesn't exist");

	}

	// returns a vector of edges incident to vertex whose
	// id is strVertexUniqueID [1 pt]
	public Vector<Edge> incidentEdges(StringBuffer strVertexUniqueID) throws GraphException {
		Vector<Edge> incidentEdgesVectors = new Vector<Edge>();
		for (Edge edge : edgeList) {
			if (edge.get_vertex1ID().toString().equals(strVertexUniqueID.toString())
					|| edge.get_vertex2ID().toString().equals(strVertexUniqueID.toString()))
				incidentEdgesVectors.add(edge);
		}
		return incidentEdgesVectors;
	}

	// returns all vertices in the graph [1 pt]
	public Vector<Vertex> vertices() throws GraphException {
		return new Vector<Vertex>(vertices);
	}

	// returns all edges in the graph [1 pt]
	public Vector<Edge> edges() throws GraphException {
		return new Vector<Edge>(edgeList);
	}

	// returns an array of the two end vertices of the
	// passed edge [1 pt]
	public Vertex[] endVertices(StringBuffer strEdgeUniqueID) throws GraphException {
		Vertex[] endVerticesArray = new Vertex[2];
		StringBuffer[] vertexIDs = new StringBuffer[2];
		vertexIDs[0] = null;

		for (Edge edge : edgeList) {
			if (edge._strUniqueID.toString().equals(strEdgeUniqueID.toString()))
				vertexIDs[0] = edge.get_vertex1ID();
			vertexIDs[1] = edge.get_vertex2ID();
		}

		if (vertexIDs[0] == null)
			throw new GraphException("Edge doesn't exist");

		for (Vertex vertex : vertices) {
			if (vertex._strUniqueID.toString().equals(vertexIDs[0].toString()))
				endVerticesArray[0] = vertex;

			if (vertex._strUniqueID.toString().equals(vertexIDs[1].toString()))
				endVerticesArray[1] = vertex;

		}
		return endVerticesArray;
	}

	// returns the vertex opposite of another vertex [1 pt]
	public Vertex opposite(StringBuffer strVertexUniqueID, StringBuffer strEdgeUniqueID) throws GraphException {
		StringBuffer edgeID = null;
		StringBuffer vertexID = null;

		for (Edge edge : edgeList) {
			if (edge._strUniqueID.toString().equals(strEdgeUniqueID.toString())) {
				edgeID = edge._strUniqueID;
				if (edge.get_vertex1ID().toString().equals(strEdgeUniqueID.toString()))
					vertexID = edge.get_vertex2ID();
				else if (edge.get_vertex2ID().toString().equals(strEdgeUniqueID.toString()))
					vertexID = edge.get_vertex1ID();
				else
					throw new GraphException("Vertex given is not associated with this edge");

			}
		}
		
		if (edgeID == null)
			throw new GraphException("Edge doesn't exist");

		for (Vertex vertex : vertices) {
			if (vertex._strUniqueID.toString().equals(vertexID.toString()))
				return vertex;
		}
		
		return null;
	}

	// performs depth first search starting from passed vertex
	// visitor is called on each vertex and edge visited. [12 pts]
	public void dfs(StringBuffer strStartVertexUniqueID, Visitor visitor) throws GraphException {

	}

	// performs breadth first search starting from passed vertex
	// visitor is called on each vertex and edge visited. [17 pts]
	public void bfs(StringBuffer strStartVertexUniqueID, Visitor visitor) throws GraphException {

	}

	// returns a path between start vertex and end vertex
	// if exists using dfs. [18 pts]
	public Vector<PathSegment> pathDFS(StringBuffer strStartVertexUniqueID, StringBuffer strEndVertexUniqueID)
			throws GraphException {
		return null;

	}

	// finds the closest pair of vertices using divide and conquer
	// algorithm. Use X and Y attributes in each vertex. [30 pts]
	public Vertex[] closestPair() throws GraphException {
		return null;

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
	public Vector<Vector<PathSegment>> findShortestPathBF(StringBuffer strStartVertexUniqueID) throws GraphException {
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