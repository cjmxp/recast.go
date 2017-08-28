package recast

type Heightfield struct {
	/** The width of the heightfield. (Along the x-axis in cell units.) */
	width int
	/** The height of the heightfield. (Along the z-axis in cell units.) */
	height int
	/** The minimum bounds in world space. [(x, y, z)] */
	bmin []float32
	/** The maximum bounds in world space. [(x, y, z)] */
	bmax []float32
	/** The size of each cell. (On the xz-plane.) */
	cs float32
	/** The height of each cell. (The minimum increment along the y-axis.) */
	ch float32
	/** Heightfield of spans (width*height). */
	spans []*Span
}

func (this *Heightfield) Init(width int, height int, bmin []float32, bmax []float32, cs float32, ch float32) {
	this.width = width
	this.height = height
	this.bmin = []float32{}
	this.bmax = []float32{}
	this.bmin = append(this.bmin, bmin...)
	this.bmax = append(this.bmax, bmax...)
	this.cs = cs
	this.ch = ch
	this.spans = make([]*Span, width*height)
}
