package detour

type StraightPathItem struct {
	pos   []float32
	flags int
	ref   int64
}

func (this *StraightPathItem) Init(pos []float32, flags int, ref int64) {
	this.pos = append(this.pos, pos...)
	this.flags = flags
	this.ref = ref
}
func (this *StraightPathItem) GetPos() []float32 {
	return this.pos
}
func (this *StraightPathItem) GetFlags() int {
	return this.flags
}
func (this *StraightPathItem) GetRef() int64 {
	return this.ref
}
