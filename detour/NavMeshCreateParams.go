package detour

type NavMeshCreateParams struct {
	/// @name Polygon Mesh Attributes
	/// Used to create the base navigation graph.
	/// See #rcPolyMesh for details related to these attributes.
	Verts     []int ///< The polygon mesh vertices. [(x, y, z) * #vertCount] [Unit: vx]
	VertCount int   ///< The number vertices in the polygon mesh. [Limit: >= 3]
	Polys     []int ///< The polygon data. [Size: #polyCount * 2 * #nvp]
	PolyFlags []int ///< The user defined flags assigned to each polygon. [Size: #polyCount]
	PolyAreas []int ///< The user defined area ids assigned to each polygon. [Size: #polyCount]
	PolyCount int   ///< Number of polygons in the mesh. [Limit: >= 1]
	Nvp       int   ///< Number maximum number of vertices per polygon. [Limit: >= 3]
	/// @name Height Detail Attributes (Optional)
	/// See #rcPolyMeshDetail for details related to these attributes.
	DetailMeshes     []int     ///< The height detail sub-mesh data. [Size: 4 * #polyCount]
	DetailVerts      []float32 ///< The detail mesh vertices. [Size: 3 * #detailVertsCount] [Unit: wu]
	DetailVertsCount int       ///< The number of vertices in the detail mesh.
	DetailTris       []int     ///< The detail mesh triangles. [Size: 4 * #detailTriCount]
	DetailTriCount   int       ///< The number of triangles in the detail mesh.
	/// @name Off-Mesh Connections Attributes (Optional)
	/// Used to define a custom point-to-point edge within the navigation graph, an
	/// off-mesh connection is a user defined traversable connection made up to two vertices,
	/// at least one of which resides within a navigation mesh polygon.
	/// Off-mesh connection vertices. [(ax, ay, az, bx, by, bz) * #offMeshConCount] [Unit: wu]
	OffMeshConVerts []float32
	/// Off-mesh connection radii. [Size: #offMeshConCount] [Unit: wu]
	OffMeshConRad []float32
	/// User defined flags assigned to the off-mesh connections. [Size: #offMeshConCount]
	OffMeshConFlags []int
	/// User defined area ids assigned to the off-mesh connections. [Size: #offMeshConCount]
	OffMeshConAreas []int
	/// The permitted travel direction of the off-mesh connections. [Size: #offMeshConCount]
	///
	/// 0 = Travel only from endpoint A to endpoint B.<br/>
	/// #DT_OFFMESH_CON_BIDIR = Bidirectional travel.
	OffMeshConDir []int
	/// The user defined ids of the off-mesh connection. [Size: #offMeshConCount]
	OffMeshConUserID []int
	/// The number of off-mesh connections. [Limit: >= 0]
	OffMeshConCount int
	/// @}
	/// @name Tile Attributes
	/// @note The tile grid/layer data can be left at zero if the destination is a single tile mesh.
	/// @{
	UserId    int       ///< The user defined id of the tile.
	TileX     int       ///< The tile's x-grid location within the multi-tile destination mesh. (Along the x-axis.)
	TileY     int       ///< The tile's y-grid location within the multi-tile desitation mesh. (Along the z-axis.)
	TileLayer int       ///< The tile's layer within the layered destination mesh. [Limit: >= 0] (Along the y-axis.)
	Bmin      []float32 ///< The minimum bounds of the tile. [(x, y, z)] [Unit: wu]
	Bmax      []float32 ///< The maximum bounds of the tile. [(x, y, z)] [Unit: wu]
	/// @name General Configuration Attributes
	WalkableHeight float32 ///< The agent height. [Unit: wu]
	WalkableRadius float32 ///< The agent radius. [Unit: wu]
	WalkableClimb  float32 ///< The agent maximum traversable ledge. (Up/Down) [Unit: wu]
	Cs             float32 ///< The xz-plane cell size of the polygon mesh. [Limit: > 0] [Unit: wu]
	Ch             float32 ///< The y-axis cell height of the polygon mesh. [Limit: > 0] [Unit: wu]
	/// True if a bounding volume tree should be built for the tile.
	/// @note The BVTree is not normally needed for layered navigation meshes.
	BuildBvTree bool
}
