package detour

type Link struct {
	/** Neighbour reference. (The neighbor that is linked to.) */
	ref int64
	/** Index of the next link. */
	next int
	/** Index of the polygon edge that owns this link. */
	edge int
	/** If a boundary link, defines on which side the link is. */
	side int
	/** If a boundary link, defines the minimum sub-edge area. */
	bmin int
	/** If a boundary link, defines the maximum sub-edge area. */
	bmax int
}
