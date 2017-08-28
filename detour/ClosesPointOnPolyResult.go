package detour

type ClosesPointOnPolyResult struct {
	posOverPoly bool
	closest     []float32
}

/** Returns true if the position is over the polygon. */
func (this *ClosesPointOnPolyResult) isPosOverPoly() bool {
	return this.posOverPoly
}

/** Returns the closest point on the polygon. [(x, y, z)] */
func (this *ClosesPointOnPolyResult) getClosest() []float32 {
	return this.closest
}
