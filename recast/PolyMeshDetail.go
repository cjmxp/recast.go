package recast

type PolyMeshDetail struct {
	/** The sub-mesh data. [Size: 4*#nmeshes] */
	meshes []int
	/** The mesh vertices. [Size: 3*#nverts] */
	verts []float32
	/** The mesh triangles. [Size: 4*#ntris] */
	tris []int
	/** The number of sub-meshes defined by #meshes. */
	nmeshes int
	/** The number of vertices in #verts. */
	nverts int
	/** The number of triangles in #tris. */
	ntris int
}

func (this *PolyMeshDetail) Get_nmeshes() int {
	return this.nmeshes
}
func (this *PolyMeshDetail) Get_nverts() int {
	return this.nverts
}
func (this *PolyMeshDetail) Get_ntris() int {
	return this.ntris
}
func (this *PolyMeshDetail) Get_verts() []float32 {
	return this.verts
}
func (this *PolyMeshDetail) Get_meshes() []int {
	return this.meshes
}
func (this *PolyMeshDetail) Get_tris() []int {
	return this.tris
}
