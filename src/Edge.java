public class Edge {
	protected StringBuffer _strUniqueID, // a unique id identifying edge
			 _strData, // data associated with this edge.
			 _vertex1ID,
			 _vertex2ID;
	// Data could be name of edge or
	// any meaningful property for
	// an edge.
	
	protected int _nEdgeCost; // cost of traversing this edge


	public Edge(StringBuffer strVertex1UniqueID, StringBuffer strVertex2UniqueID, StringBuffer strEdgeUniqueID,
			StringBuffer strEdgeData, int nEdgeCost) {
		// TODO Auto-generated constructor stub
			this._strUniqueID = strEdgeUniqueID ; 
			this._strData = strEdgeData;
			_vertex1ID = strVertex1UniqueID;
			_vertex2ID = strVertex2UniqueID;
			this._nEdgeCost = nEdgeCost;

	}

	public StringBuffer get_vertex1ID() {
		return _vertex1ID;
	}


	public StringBuffer get_vertex2ID() {
		return _vertex2ID;
	}

	public StringBuffer getUniqueID() {
		return _strUniqueID;
	}

	public StringBuffer getData() {
		return _strData;
	}

	public int getCost() {
		return _nEdgeCost;
	}
}