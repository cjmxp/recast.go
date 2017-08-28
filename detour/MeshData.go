package detour

type MeshData struct {
	/** The tile header. */
	header *MeshHeader
	/** The tile vertices. [Size: MeshHeader::vertCount] */
	verts []float32
	/** The tile polygons. [Size: MeshHeader::polyCount] */
	polys []*Poly
	/** The tile's detail sub-meshes. [Size: MeshHeader::detailMeshCount] */
	detailMeshes []*PolyDetail
	/** The detail mesh's unique vertices. [(x, y, z) * MeshHeader::detailVertCount] */
	detailVerts []float32
	/** The detail mesh's triangles. [(vertA, vertB, vertC) * MeshHeader::detailTriCount] */
	detailTris []int
	/** The tile bounding volume nodes. [Size: MeshHeader::bvNodeCount] (Will be null if bounding volumes are disabled.) */
	bvTree []*BVNode
	/** The tile off-mesh connections. [Size: MeshHeader::offMeshConCount] */
	offMeshCons []*OffMeshConnection
}

func (this *MeshData) Init() {
	this.verts = []float32{}
	this.polys = []*Poly{}
	this.detailMeshes = []*PolyDetail{}
	this.detailVerts = []float32{}
	this.detailTris = []int{}
	this.bvTree = []*BVNode{}
	this.offMeshCons = []*OffMeshConnection{}
}
