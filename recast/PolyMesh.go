package recast

type PolyMesh struct {
	/** The mesh vertices. [Form: (x, y, z) * #nverts] */
	verts []int
	/** Polygon and neighbor data. [Length: #maxpolys * 2 * #nvp] */
	polys []int
	/** The region id assigned to each polygon. [Length: #maxpolys] */
	regs []int
	/** The area id assigned to each polygon. [Length: #maxpolys] */
	areas []int
	/** The number of vertices. */
	nverts int
	/** The number of polygons. */
	npolys int
	/** The maximum number of vertices per polygon. */
	nvp int
	/** The number of allocated polygons. */
	maxpolys int
	/** The user defined flags for each polygon. [Length: #maxpolys] */
	flags []int
	/** The minimum bounds in world space. [(x, y, z)] */
	bmin []float32
	/** The maximum bounds in world space. [(x, y, z)] */
	bmax []float32
	/** The size of each cell. (On the xz-plane.) */
	cs float32
	/** The height of each cell. (The minimum increment along the y-axis.) */
	ch float32
	/** The AABB border size used to generate the source data from which the mesh was derived. */
	borderSize int
	/** The max error of the polygon edges in the mesh. */
	maxEdgeError float32
}

func (this *PolyMesh) Get_bmin() []float32 {
	return this.bmin
}
func (this *PolyMesh) Get_bmax() []float32 {
	return this.bmax
}
func (this *PolyMesh) Get_verts() []int {
	return this.verts
}
func (this *PolyMesh) Get_nverts() int {
	return this.nverts
}
func (this *PolyMesh) Get_polys() []int {
	return this.polys
}
func (this *PolyMesh) Get_npolys() int {
	return this.npolys
}
func (this *PolyMesh) Get_areas() []int {
	return this.areas
}
func (this *PolyMesh) Get_flags() []int {
	return this.flags
}
func (this *PolyMesh) Get_nvp() int {
	return this.nvp
}
