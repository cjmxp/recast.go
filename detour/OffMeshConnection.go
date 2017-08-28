package detour

type OffMeshConnection struct {
	/** The endpoints of the connection. [(ax, ay, az, bx, by, bz)] */
	pos []float32
	/** The radius of the endpoints. [Limit: >= 0] */
	rad float32
	/** The polygon reference of the connection within the tile. */
	poly int
	/**
	 * Link flags.
	 *
	 * @note These are not the connection's user defined flags. Those are assigned via the connection's Poly definition.
	 *       These are link flags used for internal purposes.
	 */
	flags int
	/** End point side. */
	side int
	/** The id of the offmesh connection. (User assigned when the navigation mesh is built.) */
	userId int
}

func (this *OffMeshConnection) Init() {
	this.pos = make([]float32, 6)
}
