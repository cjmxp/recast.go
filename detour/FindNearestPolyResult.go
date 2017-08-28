package detour

type FindNearestPolyResult struct {
	nearestRef int64
	nearestPos []float32
}

/** Returns the reference id of the nearest polygon. */
func (this *FindNearestPolyResult) GetNearestRef() int64 {
	return this.nearestRef
}

/** Returns the nearest point on the polygon. [opt] [(x, y, z)] */
func (this *FindNearestPolyResult) GetNearestPos() []float32 {
	return this.nearestPos
}
