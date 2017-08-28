package detour

type PolyDetail struct {
	/** The offset of the vertices in the MeshTile::detailVerts array. */
	vertBase int
	/** The offset of the triangles in the MeshTile::detailTris array. */
	triBase int
	/** The number of vertices in the sub-mesh. */
	vertCount int
	/** The number of triangles in the sub-mesh. */
	triCount int
}
