package detour

type QueryFilter struct {
	excludeFlags int
	includeFlags int
	areaCost     []float32
}

func (this *QueryFilter) Init() {
	this.includeFlags = 0xffff
	this.excludeFlags = 0
	this.areaCost = make([]float32, DT_MAX_AREAS)
	for i := 0; i < DT_MAX_AREAS; i++ {
		this.areaCost[i] = 1.0
	}
}
func (this *QueryFilter) passFilter(ref int64, tile *MeshTile, poly *Poly) bool {
	return (poly.flags&this.includeFlags) != 0 && (poly.flags&this.excludeFlags) == 0
}
func (this *QueryFilter) getCost(pa []float32, pb []float32, prevRef int64, prevTile *MeshTile, prevPoly *Poly, curRef int64, curTile *MeshTile, curPoly *Poly, nextRef int64, nextTile *MeshTile, nextPoly *Poly) float32 {
	return vDist(pa, pb, 0) * this.areaCost[curPoly.getArea()]
}
