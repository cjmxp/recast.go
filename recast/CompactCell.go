package recast

type CompactCell struct {
	/** Index to the first span in the column. */
	index int
	/** Number of spans in the column. */
	count int
}
