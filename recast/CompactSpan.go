package recast

type CompactSpan struct {
	/** The lower extent of the span. (Measured from the heightfield's base.) */
	y int
	/** The id of the region the span belongs to. (Or zero if not in a region.) */
	reg int
	/** Packed neighbor connection data. */
	con int
	/** The height of the span.  (Measured from #y.) */
	h int
}
