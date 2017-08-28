package recast

type Span struct {
	/** The lower limit of the span. [Limit: < #smax] */
	smin int
	/** The upper limit of the span. [Limit: <= #RC_SPAN_MAX_HEIGHT] */
	smax int
	/** The area id assigned to the span. */
	area int
	/** The next span higher up in column. */
	next *Span
}
