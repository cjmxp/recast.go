package recast

import "detour"

type RecastBuilder struct {
	pmesh *PolyMesh
	dmesh *PolyMeshDetail
}

func (this *RecastBuilder) getMesh() *PolyMesh {
	return this.pmesh
}

func (this *RecastBuilder) getMeshDetail() *PolyMeshDetail {
	return this.dmesh
}

func (this *RecastBuilder) BuildTile(geom *InputGeom, cfg *RecastConfig, mesh *detour.NavMesh, agentHeight float32, agentRadius float32, agentMaxClimb float32) {
	tw, th := calcTileCount(geom.bmin, geom.bmax, cfg.Cs, cfg.TileSize)
	for x := 0; x < tw; x++ {
		for y := 0; y < th; y++ {
			bcfg := &RecastBuilderConfig{}
			bcfg.Init(cfg, geom.bmin, geom.bmax, x, y, true)
			rt := this.Build(geom, bcfg)
			pmesh := rt.GetMesh()
			dmesh := rt.GetMeshDetail()
			params := this.getNavMeshCreateParams(cfg, pmesh, dmesh, agentHeight, agentRadius, agentMaxClimb)
			if data := detour.CreateTileNavMeshData(params, x, y); data != nil {
				mesh.AddTile(data, 0, 0)
			}
		}
	}
}
func (this *RecastBuilder) getNavMeshCreateParams(rcConfig *RecastConfig, pmesh *PolyMesh, dmesh *PolyMeshDetail, agentHeight float32, agentRadius float32, agentMaxClimb float32) *detour.NavMeshCreateParams {
	params := &detour.NavMeshCreateParams{}
	params.Verts = pmesh.verts
	params.VertCount = pmesh.nverts
	params.Polys = pmesh.polys
	params.PolyAreas = pmesh.areas
	params.PolyFlags = pmesh.flags
	params.PolyCount = pmesh.npolys
	params.Nvp = pmesh.nvp
	if dmesh != nil {
		params.DetailMeshes = dmesh.meshes
		params.DetailVerts = dmesh.verts
		params.DetailVertsCount = dmesh.nverts
		params.DetailTris = dmesh.tris
		params.DetailTriCount = dmesh.ntris
	}
	params.WalkableHeight = agentHeight
	params.WalkableRadius = agentRadius
	params.WalkableClimb = agentMaxClimb
	params.Bmin = pmesh.bmin
	params.Bmax = pmesh.bmax
	params.Cs = rcConfig.Cs
	params.Ch = rcConfig.Ch
	params.BuildBvTree = true
	return params
}

func (this *RecastBuilder) Build(geom *InputGeom, bcfg *RecastBuilderConfig) *RecastBuilderResult {
	cfg := bcfg.cfg
	chf := this.buildCompactHeightfield(geom, bcfg)
	// Partition the heightfield so that we can use simple algorithm later
	// to triangulate the walkable areas.
	// There are 3 martitioning methods, each with some pros and cons:
	// 1) Watershed partitioning
	// - the classic Recast partitioning
	// - creates the nicest tessellation
	// - usually slowest
	// - partitions the heightfield into nice regions without holes or
	// overlaps
	// - the are some corner cases where this method creates produces holes
	// and overlaps
	// - holes may appear when a small obstacles is close to large open area
	// (triangulation can handle this)
	// - overlaps may occur if you have narrow spiral corridors (i.e
	// stairs), this make triangulation to fail
	// * generally the best choice if you precompute the nacmesh, use this
	// if you have large open areas
	// 2) Monotone partioning
	// - fastest
	// - partitions the heightfield into regions without holes and overlaps
	// (guaranteed)
	// - creates long thin polygons, which sometimes causes paths with
	// detours
	// * use this if you want fast navmesh generation
	// 3) Layer partitoining
	// - quite fast
	// - partitions the heighfield into non-overlapping regions
	// - relies on the triangulation code to cope with holes (thus slower
	// than monotone partitioning)
	// - produces better triangles than monotone partitioning
	// - does not have the corner cases of watershed partitioning
	// - can be slow and create a bit ugly tessellation (still better than
	// monotone)
	// if you have large open areas with small obstacles (not a problem if
	// you use tiles)
	// * good choice to use for tiled navmesh with medium and small sized
	// tiles
	if cfg.PartitionType == WATERSHED {
		// Prepare for region partitioning, by calculating distance field
		// along the walkable surface.
		BuildDistanceField(chf)
		// Partition the walkable surface into simple regions without holes.
		BuildRegions(chf, bcfg.borderSize, cfg.MinRegionArea, cfg.MergeRegionArea)
	}
	//
	// Step 5. Trace and simplify region contours.
	//
	// Create contours.
	cset := BuildContours(chf, cfg.MaxSimplificationError, cfg.MaxEdgeLen, RC_CONTOUR_TESS_WALL_EDGES)
	//
	// Step 6. Build polygons mesh from contours.
	//
	pmesh := BuildPolyMesh(cset, cfg.MaxVertsPerPoly)
	//
	// Step 7. Create detail mesh which allows to access approximate height
	// on each polygon.
	//
	dmesh := BuildPolyMeshDetail(pmesh, chf, cfg.DetailSampleDist, cfg.DetailSampleMaxError)
	rbr := &RecastBuilderResult{}
	rbr.Init(pmesh, dmesh)
	return rbr
}
func (this *RecastBuilder) buildCompactHeightfield(geom *InputGeom, bcfg *RecastBuilderConfig) *CompactHeightfield {
	cfg := bcfg.cfg
	//
	// Step 2. Rasterize input polygon soup.
	//

	// Allocate voxel heightfield where we rasterize our input data to.
	solid := &Heightfield{}
	solid.Init(bcfg.width, bcfg.height, bcfg.bmin, bcfg.bmax, cfg.Cs, cfg.Ch)
	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to
	// process.

	// Find triangles which are walkable based on their slope and rasterize
	// them.
	// If your input data is multiple meshes, you can transform them here,
	// calculate
	// the are type for each of the meshes and rasterize them.
	verts := geom.GetVerts()
	tiled := cfg.TileSize > 0
	totaltris := 0
	if tiled {
		chunkyMesh := geom.GetChunkyMesh()
		tbmin := []float32{bcfg.bmin[0], bcfg.bmin[2]}
		tbmax := []float32{bcfg.bmax[0], bcfg.bmax[2]}
		nodes := chunkyMesh.GetChunksOverlappingRect(tbmin, tbmax)
		for i := 0; i < len(nodes); i++ {
			node := nodes[i]
			tris := node.tris
			ntris := len(tris) / 3
			totaltris += ntris
			m_triareas := MarkWalkableTriangles(cfg.WalkableSlopeAngle, verts, tris, ntris)
			RasterizeTriangles(verts, tris, m_triareas, ntris, solid, cfg.WalkableClimb)
		}
	} else {
		tris := geom.GetTris()
		ntris := len(tris) / 3
		m_triareas := MarkWalkableTriangles(cfg.WalkableSlopeAngle, verts, tris, ntris)
		totaltris = ntris
		RasterizeTriangles(verts, tris, m_triareas, ntris, solid, cfg.WalkableClimb)
	}
	//
	// Step 3. Filter walkables surfaces.
	//

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	FilterLowHangingWalkableObstacles(cfg.WalkableClimb, solid)
	FilterLedgeSpans(cfg.WalkableHeight, cfg.WalkableClimb, solid)
	FilterWalkableLowHeightSpans(cfg.WalkableHeight, solid)
	//
	// Step 4. Partition walkable surface to simple regions.
	//

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	chf := BuildCompactHeightfield(cfg.WalkableHeight, cfg.WalkableClimb, solid)
	// Erode the walkable area by agent radius.
	ErodeWalkableArea(cfg.WalkableRadius, chf)
	// (Optional) Mark areas.
	cv := geom.GetConvexVolumes()
	for i := 0; i < len(cv); i++ {
		markConvexPolyArea(cv[i].verts, cv[i].hmin, cv[i].hmax, cv[i].area, chf)
	}
	return chf
}
