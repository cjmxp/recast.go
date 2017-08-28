// TRecast project TRecast.go
package main

import (
	"detour"
	"fmt"
	"os"
	"recast"
	"strconv"
	"time"
)

func main() {
	//testBuild("dungeon", recast.WATERSHED, 52, 16, 15, 223, 118, 118, 512, 289)
	Export("test_myword")

}
func testBuild(filename string, partitionType int, expDistance int, expRegions int, expContours int, expVerts int, expPolys int, expDetMeshes int, expDetVerts int, expDetTRis int) {
	geom := (&recast.ObjImporter{}).Load(filename + ".obj")
	time1 := time.Now().UnixNano()
	bmin := geom.GetMeshBoundsMin()
	bmax := geom.GetMeshBoundsMax()
	verts := geom.GetVerts()
	tris := geom.GetTris()
	ntris := len(tris) / 3
	//
	// Step 1. Initialize build config.
	//
	cfg := &recast.RecastConfig{}
	// Init build configuration from GUI
	cfg.Init(recast.WATERSHED, 0.3, 0.2, 2.0, 0.6, 0.9, 45.0, 8.0, 20.0, 12.0, 1.3, 6.0, 6.0, 1.0, 0)
	bcfg := &recast.RecastBuilderConfig{}
	bcfg.Init(cfg, bmin, bmax, 0, 0, false)
	//
	// Step 2. Rasterize input polygon soup.
	//
	m_solid := &recast.Heightfield{}
	// Allocate voxel heightfield where we rasterize our input data to.
	m_solid.Init(bcfg.GetWidth(), bcfg.GetHeight(), bcfg.GetBmin(), bcfg.GetBmax(), cfg.Cs, cfg.Ch)
	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to
	// process.

	// Find triangles which are walkable based on their slope and rasterize
	// them.
	// If your input data is multiple meshes, you can transform them here,
	// calculate
	// the are type for each of the meshes and rasterize them.
	m_triareas := recast.MarkWalkableTriangles(cfg.WalkableSlopeAngle, verts, tris, ntris)
	recast.RasterizeTriangles(verts, tris, m_triareas, ntris, m_solid, cfg.WalkableClimb)

	//
	// Step 3. Filter walkables surfaces.
	//

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	recast.FilterLowHangingWalkableObstacles(cfg.WalkableClimb, m_solid)
	recast.FilterLedgeSpans(cfg.WalkableHeight, cfg.WalkableClimb, m_solid)
	recast.FilterWalkableLowHeightSpans(cfg.WalkableHeight, m_solid)

	//
	// Step 4. Partition walkable surface to simple regions.
	//

	// Compact the heightfield so that it is faster to handle from now on.
	// This will result more cache coherent data as well as the neighbours
	// between walkable cells will be calculated.
	m_chf := recast.BuildCompactHeightfield(cfg.WalkableHeight, cfg.WalkableClimb, m_solid)
	// Erode the walkable area by agent radius.
	recast.ErodeWalkableArea(cfg.WalkableRadius, m_chf)
	// (Optional) Mark areas.
	/*
	 * ConvexVolume vols = m_geom->getConvexVolumes(); for (int i = 0; i < m_geom->getConvexVolumeCount(); ++i)
	 * rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned
	 * char)vols[i].area, *m_chf);
	 */

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
	if cfg.PartitionType == recast.WATERSHED {
		// Prepare for region partitioning, by calculating distance field
		// along the walkable surface.
		recast.BuildDistanceField(m_chf)
		// Partition the walkable surface into simple regions without holes.
		recast.BuildRegions(m_chf, 0, cfg.MinRegionArea, cfg.MergeRegionArea)
	}
	//
	// Step 5. Trace and simplify region contours.
	//

	// Create contours.
	m_cset := recast.BuildContours(m_chf, cfg.MaxSimplificationError, cfg.MaxEdgeLen, recast.RC_CONTOUR_TESS_WALL_EDGES)
	//
	// Step 6. Build polygons mesh from contours.
	//
	// Build polygon navmesh from the contours.
	m_pmesh := recast.BuildPolyMesh(m_cset, cfg.MaxVertsPerPoly)
	//
	// Step 7. Create detail mesh which allows to access approximate height
	// on each polygon.
	//
	m_dmesh := recast.BuildPolyMeshDetail(m_pmesh, m_chf, cfg.DetailSampleDist, cfg.DetailSampleMaxError)
	fmt.Println((time.Now().UnixNano()-time1)/int64(time.Millisecond), "ms")
	saveObj("C:/MyWork/GameBoxServer/server/src/TRecast", filename, cfg.PartitionType, m_dmesh)
}
func saveObj(path string, name string, m_Type int, m_dmesh *recast.PolyMeshDetail) {
	if file, e := os.OpenFile(path+"/"+name+"_"+strconv.Itoa(m_Type)+".obj", os.O_CREATE|os.O_WRONLY|os.O_TRUNC, os.ModePerm); e == nil {
		verts := m_dmesh.Get_verts()
		for v := 0; v < m_dmesh.Get_nverts(); v++ {
			file.WriteString("v " + strconv.FormatFloat(float64(verts[v*3]), 'f', 6, 64) + " " + strconv.FormatFloat(float64(verts[v*3+1]), 'f', 6, 64) + " " + strconv.FormatFloat(float64(verts[v*3+2]), 'f', 6, 64) + "\n")
		}
		meshes := m_dmesh.Get_meshes()
		tris := m_dmesh.Get_tris()
		for m := 0; m < m_dmesh.Get_nmeshes(); m++ {
			vfirst := meshes[m*4]
			tfirst := meshes[m*4+2]
			for f := 0; f < meshes[m*4+3]; f++ {
				file.WriteString("f " + strconv.FormatInt(int64(vfirst+tris[(tfirst+f)*4]+1), 10) + " " + strconv.FormatInt(int64(vfirst+tris[(tfirst+f)*4+1]+1), 10) + " " + strconv.FormatInt(int64(vfirst+tris[(tfirst+f)*4+2]+1), 10) + "\n")
			}
		}
		file.Close()
	}
}
func Export(filename string) {
	geom := (&recast.ObjImporter{}).Load(filename + ".obj")
	RecastTestMeshBuilder(geom, recast.WATERSHED, 0.17, 0.1, 0.1, 0.0, 0.1, 45.0, 8, 20, 12, 1.3, 6, 6, 1.0)
	//RecastTestTileBuilder(geom, recast.WATERSHED, 1.0, 0.1, 0.1, 0.0, 0.1, 45.0, 8, 20, 12, 1.3, 6, 6, 1.0)
}
func RecastTestTileBuilder(geom *recast.InputGeom, partitionType int, cellSize float32, cellHeight float32, agentHeight float32, agentRadius float32, agentMaxClimb float32, agentMaxSlope float32, regionMinSize int, regionMergeSize int, edgeMaxLen float32, edgeMaxError float32, vertsPerPoly int, detailSampleDist float32, detailSampleMaxError float32) {
	params := &detour.NavMeshParams{}
	params.Init(geom.GetMeshBoundsMin(), 112*1.0, 112*1.0, 64, 65536)
	mesh := &detour.NavMesh{}
	mesh.InitTile(params, 6)
	rcBuilder := &recast.RecastBuilder{}
	cfg := &recast.RecastConfig{}
	cfg.Init(partitionType, cellSize, cellHeight, agentHeight, agentRadius, agentMaxClimb, agentMaxSlope, regionMinSize, regionMergeSize, edgeMaxLen, edgeMaxError, vertsPerPoly, detailSampleDist, detailSampleMaxError, 112)
	rcBuilder.BuildTile(geom, cfg, mesh, agentHeight, agentRadius, agentMaxClimb)

	query := &detour.NavMeshQuery{}
	query.Init(mesh)
	filter := &detour.QueryFilter{}
	filter.Init()

	//polyRefs := []int64{281474976710696, 281474976710773, 281474976710680, 281474976710753, 281474976710733}
	extents := []float32{2, 4, 2}
	//startPoss := [][]float32{{22.60652, 10.197294, -45.918674}, {22.331268, 10.197294, -1.0401875}, {18.694363, 15.803535, -73.090416}, {0.7453353, 10.197294, -5.94005}, {-20.651257, 5.904126, -13.712508}}
	//endPoss := [][]float32{{6.4576626, 10.197294, -18.33406}, {-5.8023443, 0.19729415, 3.008419}, {38.423977, 10.197294, -0.116066754}, {0.8635526, 10.197294, -10.31032}, {18.784092, 10.197294, 3.0543678}}
	//	startRefs := []int64{281474976710696, 281474976710773, 281474976710680, 281474976710753, 281474976710733}
	//	endRefs := []int64{281474976710721, 281474976710767, 281474976710758, 281474976710731, 281474976710772}
	/*for i := 0; i < len(polyRefs); i++ {
		startPos := startPoss[i]

		if polyRefs[i] != poly.GetNearestRef() {
			panic("polyRefs[i]!= poly.getNearestRef()")
		}
		fmt.Println(poly.GetNearestRef())
		fmt.Println(poly.GetNearestPos())
		for v := 0; v < len(startPoss[i]); v++ {
			if startPoss[i][v] != poly.GetNearestPos()[v] {
				panic("startPoss[i][v] != poly.GetNearestPos()[v]")
			}
		}
	}
	*/

	s := []float32{4.41, 0, 11.21}

	e := []float32{88.53, 0, 6.26}
	t := time.Now()
	path := []*detour.StraightPathItem{}
	for i := 0; i < 100; i++ {
		startRefs := query.FindNearestPoly(s, extents, filter).GetNearestRef()
		endRefs := query.FindNearestPoly(e, extents, filter).GetNearestRef()
		query.FindPath(startRefs, endRefs, s, e, filter)
		//_, node := query.FindPath(startRefs, endRefs, s, e, filter)
		//path = query.FindStraightPath(s, e, node, 0x7fffffff, 0)
	}

	fmt.Println("len", len(path), time.Now().Sub(t))
	/*dc := gg.NewContext(11000, 7000)
	dc.SetRGBA(1, 0, 0, 1)
	dc.SetLineWidth(1)
	for i := 0; i < len(path)-1; i++ {
		p1 := path[i].GetPos()
		p2 := path[i+1].GetPos()
		dc.DrawLine(float64(p1[0]*200), float64(p1[2]*-200), float64(p2[0]*200), float64(p2[2]*-200))
	}
	dc.Stroke()
	dc.SavePNG("path.png")*/
}
func RecastTestMeshBuilder(geom *recast.InputGeom, partitionType int, cellSize float32, cellHeight float32, agentHeight float32, agentRadius float32, agentMaxClimb float32, agentMaxSlope float32, regionMinSize int, regionMergeSize int, edgeMaxLen float32, edgeMaxError float32, vertsPerPoly int, detailSampleDist float32, detailSampleMaxError float32) {
	cfg := &recast.RecastConfig{}
	cfg.Init(partitionType, cellSize, cellHeight, agentHeight, agentRadius, agentMaxClimb, agentMaxSlope, regionMinSize, regionMergeSize, edgeMaxLen, edgeMaxError, vertsPerPoly, detailSampleDist, detailSampleMaxError, 0)
	//if detailSampleDist < 0.9 {
	//	cfg.DetailSampleDist = 0
	//}
	bcfg := &recast.RecastBuilderConfig{}
	bcfg.Init(cfg, geom.GetMeshBoundsMin(), geom.GetMeshBoundsMax(), 0, 0, false)

	rcBuilder := &recast.RecastBuilder{}
	rcResult := rcBuilder.Build(geom, bcfg)
	m_pmesh := rcResult.GetMesh()
	m_dmesh := rcResult.GetMeshDetail()
	params := &detour.NavMeshCreateParams{}
	params.Verts = m_pmesh.Get_verts()
	params.VertCount = m_pmesh.Get_nverts()
	params.Polys = m_pmesh.Get_polys()
	params.PolyAreas = m_pmesh.Get_areas()
	params.PolyFlags = m_pmesh.Get_flags()
	params.PolyCount = m_pmesh.Get_npolys()
	params.Nvp = m_pmesh.Get_nvp()
	params.DetailMeshes = m_dmesh.Get_meshes()
	params.DetailVerts = m_dmesh.Get_verts()
	params.DetailVertsCount = m_dmesh.Get_nverts()
	params.DetailTris = m_dmesh.Get_tris()
	params.DetailTriCount = m_dmesh.Get_ntris()
	params.WalkableHeight = agentHeight
	params.WalkableRadius = agentRadius
	params.WalkableClimb = agentMaxClimb
	params.Bmin = m_pmesh.Get_bmin()
	params.Bmax = m_pmesh.Get_bmax()
	params.Cs = cellSize
	params.Ch = cellHeight
	params.BuildBvTree = true

	params.OffMeshConVerts = []float32{0.1, 0.2, 0.3, 0.4, 0.5, 0.6}
	params.OffMeshConRad = []float32{0.1}
	params.OffMeshConDir = []int{1}
	params.OffMeshConAreas = []int{2}
	params.OffMeshConFlags = []int{12}
	params.OffMeshConUserID = []int{0x4567}
	params.OffMeshConCount = 1

	meshData := detour.CreateNavMeshData(params)
	navmesh := &detour.NavMesh{}
	navmesh.Init(meshData, 6, 0)
	query := &detour.NavMeshQuery{}
	query.Init(navmesh)
	filter := &detour.QueryFilter{}
	filter.Init()

	//polyRefs := []int64{281474976710696, 281474976710773, 281474976710680, 281474976710753, 281474976710733}
	extents := []float32{2, 4, 2}
	//startPoss := [][]float32{{22.60652, 10.197294, -45.918674}, {22.331268, 10.197294, -1.0401875}, {18.694363, 15.803535, -73.090416}, {0.7453353, 10.197294, -5.94005}, {-20.651257, 5.904126, -13.712508}}
	//endPoss := [][]float32{{6.4576626, 10.197294, -18.33406}, {-5.8023443, 0.19729415, 3.008419}, {38.423977, 10.197294, -0.116066754}, {0.8635526, 10.197294, -10.31032}, {18.784092, 10.197294, 3.0543678}}
	//	startRefs := []int64{281474976710696, 281474976710773, 281474976710680, 281474976710753, 281474976710733}
	//	endRefs := []int64{281474976710721, 281474976710767, 281474976710758, 281474976710731, 281474976710772}
	/*for i := 0; i < len(polyRefs); i++ {
		startPos := startPoss[i]

		if polyRefs[i] != poly.GetNearestRef() {
			panic("polyRefs[i]!= poly.getNearestRef()")
		}
		fmt.Println(poly.GetNearestRef())
		fmt.Println(poly.GetNearestPos())
		for v := 0; v < len(startPoss[i]); v++ {
			if startPoss[i][v] != poly.GetNearestPos()[v] {
				panic("startPoss[i][v] != poly.GetNearestPos()[v]")
			}
		}
	}
	*/

	s := []float32{4.41, 0, 11.21}

	e := []float32{88.53, 0, 6.26}
	t := time.Now()
	path := []*detour.StraightPathItem{}
	for i := 0; i < 1000; i++ {
		startRefs := query.FindNearestPoly(s, extents, filter).GetNearestRef()
		endRefs := query.FindNearestPoly(e, extents, filter).GetNearestRef()
		_, node := query.FindPath(startRefs, endRefs, s, e, filter)
		path = query.FindStraightPath(s, e, node, 0x7fffffff, 0)
	}

	fmt.Println("len", len(path), time.Now().Sub(t), "fps=", 1000/(time.Now().Sub(t).Nanoseconds()/int64(time.Millisecond)))
	/*dc := gg.NewContext(11000, 7000)
	dc.SetRGBA(1, 0, 0, 1)
	dc.SetLineWidth(1)
	for i := 0; i < len(path)-1; i++ {
		p1 := path[i].GetPos()
		p2 := path[i+1].GetPos()
		dc.DrawLine(float64(p1[0]*200), float64(p1[2]*-200), float64(p2[0]*200), float64(p2[2]*-200))
	}
	dc.Stroke()
	dc.SavePNG("path.png")*/

}
