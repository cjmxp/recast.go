package main

import (
	"detour"
	"recast"
	"testing"
)

var startPoss = [][]float32{{22.60652, 10.197294, -45.918674}, {22.331268, 10.197294, -1.0401875}, {18.694363, 15.803535, -73.090416}, {0.7453353, 10.197294, -5.94005}, {-20.651257, 5.904126, -13.712508}}
var endPoss = [][]float32{{6.4576626, 10.197294, -18.33406}, {-5.8023443, 0.19729415, 3.008419}, {38.423977, 10.197294, -0.116066754}, {0.8635526, 10.197294, -10.31032}, {18.784092, 10.197294, 3.0543678}}
var startRefs = []int64{281474976710696, 281474976710773, 281474976710680, 281474976710753, 281474976710733}
var endRefs = []int64{281474976710721, 281474976710767, 281474976710758, 281474976710731, 281474976710772}

func Benchmark_god(b *testing.B) {
	filter := &detour.QueryFilter{}
	filter.Init()
	query := tExport("C:/MyWork/GameBoxServer/server/src/TRecast/dungeon")
	for i := 0; i < b.N; i++ { //use b.N for looping
		for i := 0; i < 1; i++ {
			_, path := query.FindPath(startRefs[i], endRefs[i], startPoss[i], endPoss[i], filter)
			query.FindStraightPath(startPoss[i], endPoss[i], path, 0x7fffffff, 0)
		}
	}
}
func tExport(filename string) *detour.NavMeshQuery {
	geom := (&recast.ObjImporter{}).Load(filename + ".obj")
	q := tRecastTestMeshBuilder(geom, recast.WATERSHED, 0.3, 0.2, 2.0, 0.6, 0.9, 45.0, 8, 20, 12, 1.3, 6, 6, 1.0)
	return q
}
func tRecastTestMeshBuilder(geom *recast.InputGeom, partitionType int, cellSize float32, cellHeight float32, agentHeight float32, agentRadius float32, agentMaxClimb float32, agentMaxSlope float32, regionMinSize int, regionMergeSize int, edgeMaxLen float32, edgeMaxError float32, vertsPerPoly int, detailSampleDist float32, detailSampleMaxError float32) *detour.NavMeshQuery {
	cfg := &recast.RecastConfig{}
	cfg.Init(partitionType, cellSize, cellHeight, agentHeight, agentRadius, agentMaxClimb, agentMaxSlope, regionMinSize, regionMergeSize, edgeMaxLen, edgeMaxError, vertsPerPoly, detailSampleDist, detailSampleMaxError, 0)
	if detailSampleDist < 0.9 {
		cfg.DetailSampleDist = 0
	}
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
	//filter := &detour.QueryFilter{}
	//filter.Init()

	//polyRefs := []int64{281474976710696, 281474976710773, 281474976710680, 281474976710753, 281474976710733}
	//extents := []float32{2, 4, 2}

	/*for i := 0; i < len(polyRefs); i++ {
		startPos := startPoss[i]
		poly := query.FindNearestPoly(startPos, extents, filter)
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
	//spoly := query.FindNearestPoly(startPoss[1], extents, filter)
	//epoly := query.FindNearestPoly(endPoss[1], extents, filter)
	//t := time.Now().UnixNano()
	//for j := 0; j < 5000; j++ {

	//}
	//fmt.Println((time.Now().UnixNano()-t)/int64(time.Millisecond), "ms")
	return query
}
