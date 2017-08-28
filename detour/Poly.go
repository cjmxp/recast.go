package detour

type Poly struct {
	index int
	/** Index to first link in linked list. (Or #DT_NULL_LINK if there is no link.) */
	firstLink int
	/** The indices of the polygon's vertices. The actual vertices are located in MeshTile::verts. */
	verts []int
	/** Packed data representing neighbor polygons references and flags for each edge. */
	neis []int
	/** The user defined polygon flags. */
	flags int
	/** The number of vertices in the polygon. */
	vertCount int
	/**
	 * The bit packed area id and polygon type.
	 *
	 * @note Use the structure's set and get methods to access this value.
	 */
	areaAndtype int
}

func (this *Poly) Init(index, maxVertsPerPoly int) {
	this.index = index
	this.verts = make([]int, maxVertsPerPoly)
	this.neis = make([]int, maxVertsPerPoly)
}

/** Sets the user defined area id. [Limit: < #DT_MAX_AREAS] */
func (this *Poly) setArea(a int) {
	this.areaAndtype = (this.areaAndtype & 0xc0) | (a & 0x3f)
}

/** Sets the polygon type. (See: #dtPolyTypes.) */
func (this *Poly) setType(t int) {
	this.areaAndtype = (this.areaAndtype & 0x3f) | (t << 6)
}

/** Gets the user defined area id. */
func (this *Poly) getArea() int {
	return this.areaAndtype & 0x3f
}

/** Gets the polygon type. (See: #dtPolyTypes) */
func (this *Poly) getType() int {
	return this.areaAndtype >> 6
}
