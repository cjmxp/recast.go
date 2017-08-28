package detour

type MeshHeader struct {
	/** < Tile magic number. (Used to identify the data format.)*/
	magic int
	/** < Tile data format version number.*/
	version int
	/** < The x-position of the tile within the dtNavMesh tile grid. (x, y, layer)*/
	x int
	/** < The y-position of the tile within the dtNavMesh tile grid. (x, y, layer)*/
	y int
	/** < The layer of the tile within the dtNavMesh tile grid. (x, y, layer)*/
	layer int
	/** < The user defined id of the tile.*/
	userId int
	/** < The number of polygons in the tile.*/
	polyCount int
	/** < The number of vertices in the tile.*/
	vertCount int
	/** < The number of allocated links.*/
	maxLinkCount int
	/** < The number of sub-meshes in the detail mesh.*/
	detailMeshCount int
	/** The number of unique vertices in the detail mesh. (In addition to the polygon vertices.)*/
	detailVertCount int
	/** < The number of triangles in the detail mesh.*/
	detailTriCount int
	/** < The number of bounding volume nodes. (Zero if bounding volumes are disabled.)*/
	bvNodeCount int
	/** < The number of off-mesh connections.*/
	offMeshConCount int
	/** < The index of the first polygon which is an off-mesh connection.*/
	offMeshBase int
	/** < The height of the agents using the tile.*/
	walkableHeight float32
	/** < The radius of the agents using the tile.*/
	walkableRadius float32
	/** < The maximum climb height of the agents using the tile.*/
	walkableClimb float32
	/** < The minimum bounds of the tile's AABB. [(x, y, z)]*/
	bmin []float32
	/** < The maximum bounds of the tile's AABB. [(x, y, z)]*/
	bmax []float32
	/** The bounding volume quantization factor.*/
	bvQuantFactor float32
}

func (this *MeshHeader) Init() {
	this.bmin = make([]float32, 3)
	this.bmax = make([]float32, 3)
}
