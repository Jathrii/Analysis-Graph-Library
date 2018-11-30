public class Vertex {

	protected StringBuffer _strUniqueID, // a unique id identifying vertex
			_strData; // data associated with vertex
	protected int _nX, _nY; // Coordinates of vertex on some
	// map. Assume 0,0 is
	// bottom left.

	public Vertex(StringBuffer strUniqueID, StringBuffer _strData, int _nX, int _nY) {
		// TODO Auto-generated constructor stub
		this._strUniqueID = strUniqueID;
		this._strData = _strData;
		this._nX = _nX;
		this._nY = _nY;
	}

	public StringBuffer getUniqueID() {
		return _strUniqueID;
	}

	public StringBuffer getData() {
		return _strData;
	}

	public int getX() {
		return _nX;
	}

	public int getY() {
		return _nY;
	}

	@Override
	public String toString() {
		return this._strUniqueID.toString();
	}
}
