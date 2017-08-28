package detour

import "math"

func CreateTileNavMeshData(params *NavMeshCreateParams, x int, y int) *MeshData {
	data := CreateNavMeshData(params)
	if data != nil {
		data.header.x = x
		data.header.y = y
	}
	return data
}

func CreateNavMeshData(params *NavMeshCreateParams) *MeshData {
	if params.VertCount >= 0xffff {
		return nil
	}
	if params.VertCount == 0 || params.Verts == nil {
		return nil
	}
	if params.PolyCount == 0 || params.Polys == nil {
		return nil
	}
	nvp := params.Nvp
	// Classify off-mesh connection points. We store only the connections
	// whose start point is inside the tile.
	offMeshConClass := []int{}
	storedOffMeshConCount := 0
	offMeshConLinkCount := 0
	if params.OffMeshConCount > 0 {
		offMeshConClass = make([]int, params.OffMeshConCount*2)
		// Find tight heigh bounds, used for culling out off-mesh start
		// locations.
		hmin := FLOAT_MAX_VALUE
		hmax := -FLOAT_MAX_VALUE
		if params.DetailVerts != nil && params.DetailVertsCount != 0 {
			for i := 0; i < params.DetailVertsCount; i++ {
				h := params.DetailVerts[i*3+1]
				hmin = float32(math.Min(float64(hmin), float64(h)))
				hmax = float32(math.Max(float64(hmax), float64(h)))
			}
		} else {
			for i := 0; i < params.VertCount; i++ {
				iv := i * 3
				h := params.Bmin[1] + float32(params.Verts[iv+1])*params.Ch
				hmin = float32(math.Min(float64(hmin), float64(h)))
				hmax = float32(math.Max(float64(hmax), float64(h)))
			}
		}
		hmin -= params.WalkableClimb
		hmax += params.WalkableClimb
		bmin := make([]float32, 3)
		bmax := make([]float32, 3)
		vCopy(bmin, params.Bmin, 0)
		vCopy(bmax, params.Bmax, 0)
		bmin[1] = hmin
		bmax[1] = hmax
		for i := 0; i < params.OffMeshConCount; i++ {
			p0 := &VectorPtr{}
			p0.Init(params.OffMeshConVerts, (i*2+0)*3)
			p1 := &VectorPtr{}
			p1.Init(params.OffMeshConVerts, (i*2+1)*3)
			offMeshConClass[i*2+0] = classifyOffMeshPoint(p0, bmin, bmax)
			offMeshConClass[i*2+1] = classifyOffMeshPoint(p1, bmin, bmax)
			// Zero out off-mesh start positions which are not even
			// potentially touching the mesh.
			if offMeshConClass[i*2+0] == 0xff {
				if p0.Get(1) < bmin[1] || p0.Get(1) > bmax[1] {
					offMeshConClass[i*2+0] = 0
				}
			}
			// Count how many links should be allocated for off-mesh
			// connections.
			if offMeshConClass[i*2+0] == 0xff {
				offMeshConLinkCount++
			}
			if offMeshConClass[i*2+1] == 0xff {
				offMeshConLinkCount++
			}
			if offMeshConClass[i*2+0] == 0xff {
				storedOffMeshConCount++
			}
		}
	}
	// Off-mesh connectionss are stored as polygons, adjust values.
	totPolyCount := params.PolyCount + storedOffMeshConCount
	totVertCount := params.VertCount + storedOffMeshConCount*2
	// Find portal edges which are at tile borders.
	edgeCount := 0
	portalCount := 0
	for i := 0; i < params.PolyCount; i++ {
		p := i * 2 * nvp
		for j := 0; j < nvp; j++ {
			if params.Polys[p+j] == MESH_NULL_IDX {
				break
			}
			edgeCount++
			if (params.Polys[p+nvp+j] & 0x8000) != 0 {
				dir := params.Polys[p+nvp+j] & 0xf
				if dir != 0xf {
					portalCount++
				}
			}
		}
	}
	maxLinkCount := edgeCount + portalCount*2 + offMeshConLinkCount*2
	// Find unique detail vertices.
	uniqueDetailVertCount := 0
	detailTriCount := 0
	if params.DetailMeshes != nil {
		// Has detail mesh, count unique detail vertex count and use input
		// detail tri count.
		detailTriCount = params.DetailTriCount
		for i := 0; i < params.PolyCount; i++ {
			p := i * nvp * 2
			ndv := params.DetailMeshes[i*4+1]
			nv := 0
			for j := 0; j < nvp; j++ {
				if params.Polys[p+j] == MESH_NULL_IDX {
					break
				}
				nv++
			}
			ndv -= nv
			uniqueDetailVertCount += ndv
		}
	} else {
		// No input detail mesh, build detail mesh from nav polys.
		uniqueDetailVertCount = 0 // No extra detail verts.
		detailTriCount = 0
		for i := 0; i < params.PolyCount; i++ {
			p := i * nvp * 2
			nv := 0
			for j := 0; j < nvp; j++ {
				if params.Polys[p+j] == MESH_NULL_IDX {
					break
				}
				nv++
			}
			detailTriCount += nv - 2
		}
	}
	bvTreeSize := 0
	if params.BuildBvTree {
		bvTreeSize = params.PolyCount * 2
	}
	header := &MeshHeader{}
	header.Init()
	navVerts := make([]float32, 3*totVertCount)
	navPolys := make([]*Poly, totPolyCount)
	navDMeshes := make([]*PolyDetail, params.PolyCount)
	navDVerts := make([]float32, 3*uniqueDetailVertCount)
	navDTris := make([]int, 4*detailTriCount)
	navBvtree := make([]*BVNode, bvTreeSize)
	offMeshCons := make([]*OffMeshConnection, storedOffMeshConCount)
	// Store header
	header.magic = DT_NAVMESH_MAGIC
	header.version = DT_NAVMESH_VERSION
	header.x = params.TileX
	header.y = params.TileY
	header.layer = params.TileLayer
	header.userId = params.UserId
	header.polyCount = totPolyCount
	header.vertCount = totVertCount
	header.maxLinkCount = maxLinkCount
	vCopy(header.bmin, params.Bmin, 0)
	vCopy(header.bmax, params.Bmax, 0)
	header.detailMeshCount = params.PolyCount
	header.detailVertCount = uniqueDetailVertCount
	header.detailTriCount = detailTriCount
	header.bvQuantFactor = 1.0 / params.Cs
	header.offMeshBase = params.PolyCount
	header.walkableHeight = params.WalkableHeight
	header.walkableRadius = params.WalkableRadius
	header.walkableClimb = params.WalkableClimb
	header.offMeshConCount = storedOffMeshConCount
	header.bvNodeCount = bvTreeSize

	offMeshVertsBase := params.VertCount
	offMeshPolyBase := params.PolyCount
	// Store vertices
	// Mesh vertices
	for i := 0; i < params.VertCount; i++ {
		iv := i * 3
		v := i * 3
		navVerts[v] = params.Bmin[0] + float32(params.Verts[iv])*params.Cs
		navVerts[v+1] = params.Bmin[1] + float32(params.Verts[iv+1])*params.Ch
		navVerts[v+2] = params.Bmin[2] + float32(params.Verts[iv+2])*params.Cs
	}
	// Off-mesh link vertices.
	n := 0
	for i := 0; i < params.OffMeshConCount; i++ {
		// Only store connections which start from this tile.
		if offMeshConClass[i*2+0] == 0xff {
			linkv := i * 2 * 3
			v := (offMeshVertsBase + n*2) * 3
			arr := params.OffMeshConVerts[linkv : linkv+6]
			copy(navVerts[v:v+6], arr)
			n++
		}
	}
	// Store polygons
	// Mesh polys
	src := 0
	for i := 0; i < params.PolyCount; i++ {
		p := &Poly{}
		p.Init(i, nvp)
		p.vertCount = 0
		p.flags = params.PolyFlags[i]
		p.setArea(params.PolyAreas[i])
		p.setType(DT_POLYTYPE_GROUND)
		navPolys[i] = p
		for j := 0; j < nvp; j++ {
			if params.Polys[src+j] == MESH_NULL_IDX {
				break
			}
			p.verts[j] = params.Polys[src+j]
			if (params.Polys[src+nvp+j] & 0x8000) != 0 {
				// Border or portal edge.
				dir := params.Polys[src+nvp+j] & 0xf
				if dir == 0xf { // Border
					p.neis[j] = 0
				} else if dir == 0 { // Portal x-
					p.neis[j] = DT_EXT_LINK | 4
				} else if dir == 1 { // Portal z+
					p.neis[j] = DT_EXT_LINK | 2
				} else if dir == 2 { // Portal x+
					p.neis[j] = DT_EXT_LINK | 0
				} else if dir == 3 {
					p.neis[j] = DT_EXT_LINK | 6 // Portal z-
				}
			} else {
				// Normal connection
				p.neis[j] = params.Polys[src+nvp+j] + 1
			}
			p.vertCount++
		}
		src += nvp * 2
	}
	// Off-mesh connection vertices.
	n = 0
	for i := 0; i < params.OffMeshConCount; i++ {
		// Only store connections which start from this tile.
		if offMeshConClass[i*2+0] == 0xff {
			p := &Poly{}
			p.Init(offMeshPolyBase+n, nvp)
			navPolys[offMeshPolyBase+n] = p
			p.vertCount = 2
			p.verts[0] = offMeshVertsBase + n*2
			p.verts[1] = offMeshVertsBase + n*2 + 1
			p.flags = params.OffMeshConFlags[i]
			p.setArea(params.OffMeshConAreas[i])
			p.setType(DT_POLYTYPE_OFFMESH_CONNECTION)
			n++
		}
	}
	// Store detail meshes and vertices.
	// The nav polygon vertices are stored as the first vertices on each
	// mesh.
	// We compress the mesh data by skipping them and using the navmesh
	// coordinates.
	if params.DetailMeshes != nil {
		vbase := 0
		for i := 0; i < params.PolyCount; i++ {
			dtl := &PolyDetail{}
			navDMeshes[i] = dtl
			vb := params.DetailMeshes[i*4+0]
			ndv := params.DetailMeshes[i*4+1]
			nv := navPolys[i].vertCount
			dtl.vertBase = vbase
			dtl.vertCount = ndv - nv
			dtl.triBase = params.DetailMeshes[i*4+2]
			dtl.triCount = params.DetailMeshes[i*4+3]
			// Copy vertices except the first 'nv' verts which are equal to
			// nav poly verts.
			if ndv-nv != 0 {
				arr := params.DetailVerts[(vb+nv)*3 : (vb+nv)*3+3*(ndv-nv)]
				copy(navDVerts[vbase*3:vbase*3+3*(ndv-nv)], arr)
				vbase += ndv - nv
			}
		}
		// Store triangles.
		arr := params.DetailTris[:4*params.DetailTriCount]
		copy(navDTris[:4*params.DetailTriCount], arr)
	} else {
		// Create dummy detail mesh by triangulating polys.
		tbase := 0
		for i := 0; i < params.PolyCount; i++ {
			dtl := &PolyDetail{}
			navDMeshes[i] = dtl
			nv := navPolys[i].vertCount
			dtl.vertBase = 0
			dtl.vertCount = 0
			dtl.triBase = tbase
			dtl.triCount = (nv - 2)
			// Triangulate polygon (local indices).
			for j := 2; j < nv; j++ {
				t := tbase * 4
				navDTris[t+0] = 0
				navDTris[t+1] = (j - 1)
				navDTris[t+2] = j
				// Bit for each edge that belongs to poly boundary.
				navDTris[t+3] = (1 << 2)
				if j == 2 {
					navDTris[t+3] |= (1 << 0)
				}
				if j == nv-1 {
					navDTris[t+3] |= (1 << 4)
				}
				tbase++
			}
		}
	}
	// Store and create BVtree.
	// TODO: take detail mesh into account! use byte per bbox extent?
	if params.BuildBvTree {
		// Do not set header.bvNodeCount set to make it work look exactly the same as in original Detour
		header.bvNodeCount = createBVTree(params, navBvtree)
	}
	// Store Off-Mesh connections.
	n = 0
	for i := 0; i < params.OffMeshConCount; i++ {
		// Only store connections which start from this tile.
		if offMeshConClass[i*2+0] == 0xff {
			con := &OffMeshConnection{}
			con.Init()
			offMeshCons[n] = con
			con.poly = offMeshPolyBase + n
			// Copy connection end-points.
			endPts := i * 2 * 3
			arr := params.OffMeshConVerts[endPts : endPts+6]
			copy(con.pos[:6], arr)
			con.rad = params.OffMeshConRad[i]
			con.flags = 0
			if params.OffMeshConDir[i] != 0 {
				con.flags = DT_OFFMESH_CON_BIDIR
			}
			con.side = offMeshConClass[i*2+1]
			if params.OffMeshConUserID != nil {
				con.userId = params.OffMeshConUserID[i]
			}
			n++
		}
	}
	nmd := &MeshData{}
	nmd.Init()
	nmd.header = header
	nmd.verts = navVerts
	nmd.polys = navPolys
	nmd.detailMeshes = navDMeshes
	nmd.detailVerts = navDVerts
	nmd.detailTris = navDTris
	nmd.bvTree = navBvtree
	nmd.offMeshCons = offMeshCons
	return nmd
}
func classifyOffMeshPoint(pt *VectorPtr, bmin []float32, bmax []float32) int {
	outcode := 0
	if pt.Get(0) >= bmax[0] {
		outcode |= XP
	} else {
		outcode |= 0
	}
	if pt.Get(2) >= bmax[2] {
		outcode |= ZP
	} else {
		outcode |= 0
	}
	if pt.Get(0) < bmin[0] {
		outcode |= XM
	} else {
		outcode |= 0
	}
	if pt.Get(2) < bmin[2] {
		outcode |= ZM
	} else {
		outcode |= 0
	}
	switch outcode {
	case XP:
		return 0
	case XP | ZP:
		return 1
	case ZP:
		return 2
	case XM | ZP:
		return 3
	case XM:
		return 4
	case XM | ZM:
		return 5
	case ZM:
		return 6
	case XP | ZM:
		return 7
	}
	return 0xff
}
func createBVTree(params *NavMeshCreateParams, nodes []*BVNode) int {
	// Build tree
	quantFactor := 1.0 / params.Cs
	items := make([]*BVItem, params.PolyCount)
	for i := 0; i < params.PolyCount; i++ {
		it := &BVItem{}
		it.Init()
		items[i] = it
		it.i = i
		// Calc polygon bounds. Use detail meshes if available.
		if params.DetailMeshes != nil {
			vb := params.DetailMeshes[i*4+0]
			ndv := params.DetailMeshes[i*4+1]
			bmin := make([]float32, 3)
			bmax := make([]float32, 3)
			dv := vb * 3
			vCopy(bmin, params.DetailVerts, dv)
			vCopy(bmax, params.DetailVerts, dv)
			for j := 1; j < ndv; j++ {
				vMin(bmin, params.DetailVerts, dv+j*3)
				vMax(bmax, params.DetailVerts, dv+j*3)
			}
			// BV-tree uses cs for all dimensions
			it.bmin[0] = clamp_i(int(((bmin[0] - params.Bmin[0]) * quantFactor)), 0, 0xffff)
			it.bmin[1] = clamp_i(int(((bmin[1] - params.Bmin[1]) * quantFactor)), 0, 0xffff)
			it.bmin[2] = clamp_i(int(((bmin[2] - params.Bmin[2]) * quantFactor)), 0, 0xffff)
			it.bmax[0] = clamp_i(int(((bmax[0] - params.Bmin[0]) * quantFactor)), 0, 0xffff)
			it.bmax[1] = clamp_i(int(((bmax[1] - params.Bmin[1]) * quantFactor)), 0, 0xffff)
			it.bmax[2] = clamp_i(int(((bmax[2] - params.Bmin[2]) * quantFactor)), 0, 0xffff)
		} else {
			p := i * params.Nvp * 2
			it.bmin[0] = params.Verts[params.Polys[p]*3+0]
			it.bmax[0] = it.bmin[0]
			it.bmin[1] = params.Verts[params.Polys[p]*3+1]
			it.bmax[1] = it.bmin[1]
			it.bmin[2] = params.Verts[params.Polys[p]*3+2]
			it.bmax[2] = it.bmin[2]
			for j := 1; j < params.Nvp; j++ {
				if params.Polys[p+j] == MESH_NULL_IDX {
					break
				}
				x := params.Verts[params.Polys[p+j]*3+0]
				y := params.Verts[params.Polys[p+j]*3+1]
				z := params.Verts[params.Polys[p+j]*3+2]
				if x < it.bmin[0] {
					it.bmin[0] = x
				}
				if y < it.bmin[1] {
					it.bmin[1] = y
				}
				if z < it.bmin[2] {
					it.bmin[2] = z
				}
				if x > it.bmax[0] {
					it.bmax[0] = x
				}
				if y > it.bmax[1] {
					it.bmax[1] = y
				}
				if z > it.bmax[2] {
					it.bmax[2] = z
				}
			}
			// Remap y
			it.bmin[1] = int(math.Floor(float64(float32(it.bmin[1]) * params.Ch / params.Cs)))
			it.bmax[1] = int(math.Ceil(float64(float32(it.bmax[1]) * params.Ch / params.Cs)))
		}
	}
	return subdivide(items, params.PolyCount, 0, params.PolyCount, 0, nodes)
}

func subdivide(items []*BVItem, nitems int, imin int, imax int, curNode int, nodes []*BVNode) int {
	inum := imax - imin
	icur := curNode
	node := &BVNode{}
	node.Init()
	nodes[curNode] = node
	curNode++
	if inum == 1 {
		// Leaf
		node.bmin[0] = items[imin].bmin[0]
		node.bmin[1] = items[imin].bmin[1]
		node.bmin[2] = items[imin].bmin[2]

		node.bmax[0] = items[imin].bmax[0]
		node.bmax[1] = items[imin].bmax[1]
		node.bmax[2] = items[imin].bmax[2]
		node.i = items[imin].i
	} else {
		// Split
		node.bmin, node.bmax = calcExtends(items, nitems, imin, imax)
		axis := longestAxis(node.bmax[0]-node.bmin[0], node.bmax[1]-node.bmin[1], node.bmax[2]-node.bmin[2])
		isort := ISort{}
		if axis == 0 {
			// Sort along x-axis
			isort.Data = items[imin : imin+inum]
			isort.Call = func(a interface{}, b interface{}) bool {
				if a.(*BVItem).bmin[0] < b.(*BVItem).bmin[0] {
					return true
				}
				return false
			}
			isort.Stable()
			isort.Free()

		} else if axis == 1 {
			// Sort along y-axis
			isort.Data = items[imin : imin+inum]
			isort.Call = func(a interface{}, b interface{}) bool {
				if a.(*BVItem).bmin[1] < b.(*BVItem).bmin[1] {
					return true
				}

				return false
			}
			isort.Stable()
			isort.Free()
		} else {
			// Sort along z-axis
			isort.Data = items[imin : imin+inum]
			isort.Call = func(a interface{}, b interface{}) bool {
				if a.(*BVItem).bmin[2] < b.(*BVItem).bmin[2] {
					return true
				}
				return false
			}
			isort.Stable()
			isort.Free()
		}
		isplit := imin + inum/2
		// Left
		curNode = subdivide(items, nitems, imin, isplit, curNode, nodes)
		// Right
		curNode = subdivide(items, nitems, isplit, imax, curNode, nodes)
		iescape := curNode - icur
		// Negative index means escape.
		node.i = -iescape
	}
	return curNode
}

func calcExtends(items []*BVItem, nitems int, imin int, imax int) ([]int, []int) {
	bmin := make([]int, 3)
	bmax := make([]int, 3)
	bmin[0] = items[imin].bmin[0]
	bmin[1] = items[imin].bmin[1]
	bmin[2] = items[imin].bmin[2]
	bmax[0] = items[imin].bmax[0]
	bmax[1] = items[imin].bmax[1]
	bmax[2] = items[imin].bmax[2]

	for i := imin + 1; i < imax; i++ {
		it := items[i]
		if it.bmin[0] < bmin[0] {
			bmin[0] = it.bmin[0]
		}
		if it.bmin[1] < bmin[1] {
			bmin[1] = it.bmin[1]
		}
		if it.bmin[2] < bmin[2] {
			bmin[2] = it.bmin[2]
		}
		if it.bmax[0] > bmax[0] {
			bmax[0] = it.bmax[0]
		}
		if it.bmax[1] > bmax[1] {
			bmax[1] = it.bmax[1]
		}
		if it.bmax[2] > bmax[2] {
			bmax[2] = it.bmax[2]
		}
	}
	return bmin, bmax
}
func longestAxis(x, y, z int) int {
	axis := 0
	maxVal := x
	if y > maxVal {
		axis = 1
		maxVal = y
	}
	if z > maxVal {
		axis = 2
		maxVal = z
	}
	return axis
}
