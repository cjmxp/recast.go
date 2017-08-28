package recast

import (
	"math"
	"strconv"
)

func BuildPolyMesh(cset *ContourSet, nvp int) *PolyMesh {
	mesh := &PolyMesh{}
	mesh.bmin = make([]float32, 3)
	mesh.bmax = make([]float32, 3)
	copy3(mesh.bmin, 0, cset.bmin, 0)
	copy3(mesh.bmax, 0, cset.bmax, 0)
	mesh.cs = cset.cs
	mesh.ch = cset.ch
	mesh.borderSize = cset.borderSize
	mesh.maxEdgeError = cset.maxError
	maxVertices := 0
	maxTris := 0
	maxVertsPerCont := 0
	for i := 0; i < len(cset.conts); i++ {
		// Skip null contours.
		if cset.conts[i].nverts < 3 {
			continue
		}
		maxVertices += cset.conts[i].nverts
		maxTris += cset.conts[i].nverts - 2
		maxVertsPerCont = int(math.Max(float64(maxVertsPerCont), float64(cset.conts[i].nverts)))
	}
	if maxVertices >= 0xfffe {
		panic("rcBuildPolyMesh: Too many vertices " + strconv.Itoa(maxVertices))
	}
	vflags := make([]int, maxVertices)
	mesh.verts = make([]int, maxVertices*3)
	mesh.polys = make([]int, maxTris*nvp*2)
	for i := 0; i < len(mesh.polys); i++ {
		mesh.polys[i] = RC_MESH_NULL_IDX
	}
	mesh.regs = make([]int, maxTris)
	mesh.areas = make([]int, maxTris)

	mesh.nverts = 0
	mesh.npolys = 0
	mesh.nvp = nvp
	mesh.maxpolys = maxTris

	nextVert := make([]int, maxVertices)
	firstVert := make([]int, VERTEX_BUCKET_COUNT)
	for i := 0; i < VERTEX_BUCKET_COUNT; i++ {
		firstVert[i] = -1
	}
	indices := make([]int, maxVertsPerCont)
	tris := make([]int, maxVertsPerCont*3)
	polys := make([]int, (maxVertsPerCont+1)*nvp)
	tmpPoly := maxVertsPerCont * nvp
	for i := 0; i < len(cset.conts); i++ {
		cont := cset.conts[i]
		// Skip null contours.
		if cont.nverts < 3 {
			continue
		}
		// Triangulate contour
		for j := 0; j < cont.nverts; j++ {
			indices[j] = j
		}
		ntris := triangulate(cont.nverts, cont.verts, indices, tris)
		if ntris <= 0 {
			// Bad triangulation, should not happen.
			println("buildPolyMesh: Bad triangulation Contour " + strconv.Itoa(i) + ".")
			ntris = -ntris
		}
		// Add and merge vertices.
		for j := 0; j < cont.nverts; j++ {
			v := j * 4
			inv := addVertex(cont.verts[v+0], cont.verts[v+1], cont.verts[v+2], mesh.verts, firstVert, nextVert, mesh.nverts)
			indices[j] = inv[0]
			mesh.nverts = inv[1]
			if (cont.verts[v+3] & RC_BORDER_VERTEX) != 0 {
				// This vertex should be removed.
				vflags[indices[j]] = 1
			}
		}
		// Build initial polygons.
		npolys := 0
		for i := 0; i < len(polys); i++ {
			polys[i] = RC_MESH_NULL_IDX
		}
		for j := 0; j < ntris; j++ {
			t := j * 3
			if tris[t+0] != tris[t+1] && tris[t+0] != tris[t+2] && tris[t+1] != tris[t+2] {
				polys[npolys*nvp+0] = indices[tris[t+0]]
				polys[npolys*nvp+1] = indices[tris[t+1]]
				polys[npolys*nvp+2] = indices[tris[t+2]]
				npolys++
			}
		}
		if npolys == 0 {
			continue
		}
		// Merge polygons.
		if nvp > 3 {
			for {
				// Find best polygons to merge.
				bestMergeVal := 0
				bestPa, bestPb, bestEa, bestEb := 0, 0, 0, 0
				for j := 0; j < npolys-1; j++ {
					pj := j * nvp
					for k := j + 1; k < npolys; k++ {
						pk := k * nvp
						veaeb := getPolyMergeValue(polys, pj, pk, mesh.verts, nvp)
						v := veaeb[0]
						ea := veaeb[1]
						eb := veaeb[2]
						if v > bestMergeVal {
							bestMergeVal = v
							bestPa = j
							bestPb = k
							bestEa = ea
							bestEb = eb
						}
					}
				}
				if bestMergeVal > 0 {
					// Found best, merge.
					pa := bestPa * nvp
					pb := bestPb * nvp
					mergePolyVerts(polys, pa, pb, bestEa, bestEb, tmpPoly, nvp)
					lastPoly := (npolys - 1) * nvp
					if pb != lastPoly {
						arr := polys[lastPoly : lastPoly+nvp]
						copy(polys[pb:pb+nvp], arr)
					}
					npolys--
				} else {
					// Could not merge any polygons, stop.
					break
				}
			}
		}
		// Store polygons.
		for j := 0; j < npolys; j++ {
			p := mesh.npolys * nvp * 2
			q := j * nvp
			for k := 0; k < nvp; k++ {
				mesh.polys[p+k] = polys[q+k]
			}
			mesh.regs[mesh.npolys] = cont.reg
			mesh.areas[mesh.npolys] = cont.area
			mesh.npolys++
			if mesh.npolys > maxTris {
				panic("rcBuildPolyMesh: Too many polygons " + strconv.Itoa(mesh.npolys) + " (max:" + strconv.Itoa(maxTris) + ").")
			}
		}
	}
	// Remove edge vertices.
	for i := 0; i < mesh.nverts; i++ {
		if vflags[i] != 0 {
			if !canRemoveVertex(mesh, i) {
				continue
			}
			removeVertex(mesh, i, maxTris)
			// Remove vertex
			// Note: mesh.nverts is already decremented inside removeVertex()!
			// Fixup vertex flags
			for j := i; j < mesh.nverts; j++ {
				vflags[j] = vflags[j+1]
			}
			i--
		}
	}
	// Calculate adjacency.
	buildMeshAdjacency(mesh.polys, mesh.npolys, mesh.nverts, nvp)
	// Find portal edges
	if mesh.borderSize > 0 {
		w := cset.width
		h := cset.height
		for i := 0; i < mesh.npolys; i++ {
			p := i * 2 * nvp
			for j := 0; j < nvp; j++ {
				if mesh.polys[p+j] == RC_MESH_NULL_IDX {
					break
				}
				// Skip connected edges.
				if mesh.polys[p+nvp+j] != RC_MESH_NULL_IDX {
					continue
				}
				nj := j + 1
				if nj >= nvp || mesh.polys[p+nj] == RC_MESH_NULL_IDX {
					nj = 0
				}
				va := mesh.polys[p+j] * 3
				vb := mesh.polys[p+nj] * 3
				if mesh.verts[va+0] == 0 && mesh.verts[vb+0] == 0 {
					mesh.polys[p+nvp+j] = 0x8000 | 0
				} else if mesh.verts[va+2] == h && mesh.verts[vb+2] == h {
					mesh.polys[p+nvp+j] = 0x8000 | 1

				} else if mesh.verts[va+0] == w && mesh.verts[vb+0] == w {
					mesh.polys[p+nvp+j] = 0x8000 | 2
				} else if mesh.verts[va+2] == 0 && mesh.verts[vb+2] == 0 {
					mesh.polys[p+nvp+j] = 0x8000 | 3
				}
			}
		}
	}
	// Just allocate the mesh flags array. The user is resposible to fill it.
	mesh.flags = make([]int, mesh.npolys)
	if mesh.nverts > 0xffff {
		panic("rcBuildPolyMesh: The resulting mesh has too many vertices " + strconv.Itoa(mesh.nverts) + " (max " + strconv.Itoa(0xffff) + "). Data can be corrupted.")
	}
	if mesh.npolys > 0xffff {
		panic("rcBuildPolyMesh: The resulting mesh has too many polygons " + strconv.Itoa(mesh.npolys) + " (max " + strconv.Itoa(0xffff) + "). Data can be corrupted.")
	}
	return mesh
}
func buildMeshAdjacency(polys []int, npolys int, nverts int, vertsPerPoly int) {
	// Based on code by Eric Lengyel from:
	// http://www.terathon.com/code/edges.php
	maxEdgeCount := npolys * vertsPerPoly
	firstEdge := make([]int, nverts+maxEdgeCount)
	nextEdge := nverts
	edgeCount := 0
	edges := make([]*Edge, maxEdgeCount)
	for i := 0; i < nverts; i++ {
		firstEdge[i] = RC_MESH_NULL_IDX
	}
	for i := 0; i < npolys; i++ {
		t := i * vertsPerPoly * 2
		for j := 0; j < vertsPerPoly; j++ {
			if polys[t+j] == RC_MESH_NULL_IDX {
				break
			}
			v0 := polys[t+j]
			v1 := polys[t+j+1]
			if j+1 >= vertsPerPoly || polys[t+j+1] == RC_MESH_NULL_IDX {
				v1 = polys[t+0]
			}
			if v0 < v1 {
				edge := &Edge{}
				edges[edgeCount] = edge
				edge.vert = []int{v0, v1}
				edge.poly = []int{i, i}
				edge.polyEdge = []int{j, 0}
				// Insert edge
				firstEdge[nextEdge+edgeCount] = firstEdge[v0]
				firstEdge[v0] = edgeCount
				edgeCount++
			}
		}
	}
	for i := 0; i < npolys; i++ {
		t := i * vertsPerPoly * 2
		for j := 0; j < vertsPerPoly; j++ {
			if polys[t+j] == RC_MESH_NULL_IDX {
				break
			}
			v0 := polys[t+j]
			v1 := polys[t+j+1]
			if j+1 >= vertsPerPoly || polys[t+j+1] == RC_MESH_NULL_IDX {
				v1 = polys[t+0]
			}
			if v0 > v1 {
				for e := firstEdge[v1]; e != RC_MESH_NULL_IDX; e = firstEdge[nextEdge+e] {
					edge := edges[e]
					if edge.vert[1] == v0 && edge.poly[0] == edge.poly[1] {
						edge.poly[1] = i
						edge.polyEdge[1] = j
						break
					}
				}
			}
		}
	}
	// Store adjacency
	for i := 0; i < edgeCount; i++ {
		e := edges[i]
		if e.poly[0] != e.poly[1] {
			p0 := e.poly[0] * vertsPerPoly * 2
			p1 := e.poly[1] * vertsPerPoly * 2
			polys[p0+vertsPerPoly+e.polyEdge[0]] = e.poly[1]
			polys[p1+vertsPerPoly+e.polyEdge[1]] = e.poly[0]
		}
	}
}
func removeVertex(mesh *PolyMesh, rem int, maxTris int) {
	nvp := mesh.nvp
	// Count number of polygons to remove.
	numRemovedVerts := 0
	for i := 0; i < mesh.npolys; i++ {
		p := i * nvp * 2
		nv := countPolyVerts(mesh.polys, p, nvp)
		for j := 0; j < nv; j++ {
			if mesh.polys[p+j] == rem {
				numRemovedVerts++
			}
		}
	}
	nedges := 0
	edges := make([]int, numRemovedVerts*nvp*4)
	nhole := 0
	hole := make([]int, numRemovedVerts*nvp)
	nhreg := 0
	hreg := make([]int, numRemovedVerts*nvp)
	nharea := 0
	harea := make([]int, numRemovedVerts*nvp)
	for i := 0; i < mesh.npolys; i++ {
		p := i * nvp * 2
		nv := countPolyVerts(mesh.polys, p, nvp)
		hasRem := false
		for j := 0; j < nv; j++ {
			if mesh.polys[p+j] == rem {
				hasRem = true
			}
		}
		if hasRem {
			// Collect edges which does not touch the removed vertex.
			for j, k := 0, nv-1; j < nv; k, j = j, j+1 {
				if mesh.polys[p+j] != rem && mesh.polys[p+k] != rem {
					e := nedges * 4
					edges[e+0] = mesh.polys[p+k]
					edges[e+1] = mesh.polys[p+j]
					edges[e+2] = mesh.regs[i]
					edges[e+3] = mesh.areas[i]
					nedges++
				}
			}
			// Remove the polygon.
			p2 := (mesh.npolys - 1) * nvp * 2
			if p != p2 {
				arr := mesh.polys[p2 : p2+nvp]
				copy(mesh.polys[p:p+nvp], arr)
			}
			for i := p + nvp; i < p+nvp+nvp; i++ {
				mesh.polys[i] = RC_MESH_NULL_IDX
			}
			mesh.regs[i] = mesh.regs[mesh.npolys-1]
			mesh.areas[i] = mesh.areas[mesh.npolys-1]
			mesh.npolys--
			i--
		}
	}
	// Remove vertex.
	for i := rem; i < mesh.nverts-1; i++ {
		mesh.verts[i*3+0] = mesh.verts[(i+1)*3+0]
		mesh.verts[i*3+1] = mesh.verts[(i+1)*3+1]
		mesh.verts[i*3+2] = mesh.verts[(i+1)*3+2]
	}
	mesh.nverts--
	// Adjust indices to match the removed vertex layout.
	for i := 0; i < mesh.npolys; i++ {
		p := i * nvp * 2
		nv := countPolyVerts(mesh.polys, p, nvp)
		for j := 0; j < nv; j++ {
			if mesh.polys[p+j] > rem {
				mesh.polys[p+j]--
			}
		}
	}
	for i := 0; i < nedges; i++ {
		if edges[i*4+0] > rem {
			edges[i*4+0]--
		}
		if edges[i*4+1] > rem {
			edges[i*4+1]--
		}
	}
	if nedges == 0 {
		return
	}
	// Start with one vertex, keep appending connected
	// segments to the start and end of the hole.
	pushBack(edges[0], hole, nhole)
	pushBack(edges[2], hreg, nhreg)
	pushBack(edges[3], harea, nharea)
	for nedges != 0 {
		match := false
		for i := 0; i < nedges; i++ {
			ea := edges[i*4+0]
			eb := edges[i*4+1]
			r := edges[i*4+2]
			a := edges[i*4+3]
			add := false
			if hole[0] == eb {
				// The segment matches the beginning of the hole boundary.
				pushFront(ea, hole, nhole)
				pushFront(r, hreg, nhreg)
				pushFront(a, harea, nharea)
				add = true
			} else if hole[nhole-1] == ea {
				// The segment matches the end of the hole boundary.
				nhole = pushBack(eb, hole, nhole)
				nhreg = pushBack(r, hreg, nhreg)
				nharea = pushBack(a, harea, nharea)
				add = true
			}
			if add {
				// The edge segment was added, remove it.
				edges[i*4+0] = edges[(nedges-1)*4+0]
				edges[i*4+1] = edges[(nedges-1)*4+1]
				edges[i*4+2] = edges[(nedges-1)*4+2]
				edges[i*4+3] = edges[(nedges-1)*4+3]
				nedges--
				match = true
				i--
			}
		}
		if !match {
			break
		}
	}

	tris := make([]int, nhole*3)
	tverts := make([]int, nhole*4)
	thole := make([]int, nhole)
	// Generate temp vertex array for triangulation.
	for i := 0; i < nhole; i++ {
		pi := hole[i]
		tverts[i*4+0] = mesh.verts[pi*3+0]
		tverts[i*4+1] = mesh.verts[pi*3+1]
		tverts[i*4+2] = mesh.verts[pi*3+2]
		tverts[i*4+3] = 0
		thole[i] = i
	}
	// Triangulate the hole.
	ntris := triangulate(nhole, tverts, thole, tris)
	if ntris < 0 {
		ntris = -ntris
		println("removeVertex: triangulate() returned bad results.")
	}
	// Merge the hole triangles back to polygons.
	polys := make([]int, (ntris+1)*nvp)
	pregs := make([]int, ntris)
	pareas := make([]int, ntris)
	tmpPoly := ntris * nvp
	// Build initial polygons.
	npolys := 0
	for i := 0; i < ntris*nvp; i++ {
		polys[i] = RC_MESH_NULL_IDX
	}
	for j := 0; j < ntris; j++ {
		t := j * 3
		if tris[t+0] != tris[t+1] && tris[t+0] != tris[t+2] && tris[t+1] != tris[t+2] {
			polys[npolys*nvp+0] = hole[tris[t+0]]
			polys[npolys*nvp+1] = hole[tris[t+1]]
			polys[npolys*nvp+2] = hole[tris[t+2]]
			// If this polygon covers multiple region types then
			// mark it as such
			if hreg[tris[t+0]] != hreg[tris[t+1]] || hreg[tris[t+1]] != hreg[tris[t+2]] {
				pregs[npolys] = RC_MULTIPLE_REGS
			} else {
				pregs[npolys] = hreg[tris[t+0]]
			}
			pareas[npolys] = harea[tris[t+0]]
			npolys++
		}
	}
	if npolys == 0 {
		return
	}
	// Merge polygons.
	if nvp > 3 {
		for {
			// Find best polygons to merge.
			bestMergeVal := 0
			bestPa, bestPb, bestEa, bestEb := 0, 0, 0, 0
			for j := 0; j < npolys-1; j++ {
				pj := j * nvp
				for k := j + 1; k < npolys; k++ {
					pk := k * nvp
					veaeb := getPolyMergeValue(polys, pj, pk, mesh.verts, nvp)
					v := veaeb[0]
					ea := veaeb[1]
					eb := veaeb[2]
					if v > bestMergeVal {
						bestMergeVal = v
						bestPa = j
						bestPb = k
						bestEa = ea
						bestEb = eb
					}
				}
			}
			if bestMergeVal > 0 {
				// Found best, merge.
				pa := bestPa * nvp
				pb := bestPb * nvp
				mergePolyVerts(polys, pa, pb, bestEa, bestEb, tmpPoly, nvp)
				if pregs[bestPa] != pregs[bestPb] {
					pregs[bestPa] = RC_MULTIPLE_REGS
				}
				last := (npolys - 1) * nvp
				if pb != last {
					arr := polys[last : last+nvp]
					copy(polys[pb:pb+nvp], arr)
				}
				pregs[bestPb] = pregs[npolys-1]
				pareas[bestPb] = pareas[npolys-1]
				npolys--
			} else {
				// Could not merge any polygons, stop.
				break
			}
		}
	}
	// Store polygons.
	for i := 0; i < npolys; i++ {
		if mesh.npolys >= maxTris {
			break
		}
		p := mesh.npolys * nvp * 2
		for i := p; i < p+nvp*2; i++ {
			mesh.polys[i] = RC_MESH_NULL_IDX
		}
		for j := 0; j < nvp; j++ {
			mesh.polys[p+j] = polys[i*nvp+j]
		}
		mesh.regs[mesh.npolys] = pregs[i]
		mesh.areas[mesh.npolys] = pareas[i]
		mesh.npolys++
		if mesh.npolys > maxTris {
			panic("removeVertex: Too many polygons " + strconv.Itoa(mesh.npolys) + " (max:" + strconv.Itoa(maxTris) + ".")
		}
	}
}
func pushFront(v int, arr []int, an int) int {
	an++
	for i := an - 1; i > 0; i-- {
		arr[i] = arr[i-1]
	}
	arr[0] = v
	return an
}
func pushBack(v int, arr []int, an int) int {
	arr[an] = v
	an++
	return an
}
func canRemoveVertex(mesh *PolyMesh, rem int) bool {
	nvp := mesh.nvp
	// Count number of polygons to remove.
	numTouchedVerts := 0
	numRemainingEdges := 0
	for i := 0; i < mesh.npolys; i++ {
		p := i * nvp * 2
		nv := countPolyVerts(mesh.polys, p, nvp)
		numRemoved := 0
		numVerts := 0
		for j := 0; j < nv; j++ {
			if mesh.polys[p+j] == rem {
				numTouchedVerts++
				numRemoved++
			}
			numVerts++
		}
		if numRemoved != 0 {
			numRemainingEdges += numVerts - (numRemoved + 1)
		}
	}
	// There would be too few edges remaining to create a polygon.
	// This can happen for example when a tip of a triangle is marked
	// as deletion, but there are no other polys that share the vertex.
	// In this case, the vertex should not be removed.
	if numRemainingEdges <= 2 {
		return false
	}
	// Find edges which share the removed vertex.
	maxEdges := numTouchedVerts * 2
	nedges := 0
	edges := make([]int, maxEdges*3)
	for i := 0; i < mesh.npolys; i++ {
		p := i * nvp * 2
		nv := countPolyVerts(mesh.polys, p, nvp)
		// Collect edges which touches the removed vertex.
		for j, k := 0, nv-1; j < nv; k, j = j, j+1 {
			if mesh.polys[p+j] == rem || mesh.polys[p+k] == rem {
				// Arrange edge so that a=rem.
				a := mesh.polys[p+j]
				b := mesh.polys[p+k]
				if b == rem {
					a, b = b, a
				}
				// Check if the edge exists
				exists := false
				for m := 0; m < nedges; m++ {
					e := m * 3
					if edges[e+1] == b {
						// Exists, increment vertex share count.
						edges[e+2]++
						exists = true
					}
				}
				// Add new edge.
				if !exists {
					e := nedges * 3
					edges[e+0] = a
					edges[e+1] = b
					edges[e+2] = 1
					nedges++
				}
			}
		}
	}
	// There should be no more than 2 open edges.
	// This catches the case that two non-adjacent polygons
	// share the removed vertex. In that case, do not remove the vertex.
	numOpenEdges := 0
	for i := 0; i < nedges; i++ {
		if edges[i*3+2] < 2 {
			numOpenEdges++
		}
	}
	if numOpenEdges > 2 {
		return false
	}
	return true
}
func mergePolyVerts(polys []int, pa int, pb int, ea int, eb int, tmp int, nvp int) {
	na := countPolyVerts(polys, pa, nvp)
	nb := countPolyVerts(polys, pb, nvp)
	// Merge polygons.
	for i := tmp; i < tmp+nvp; i++ {
		polys[i] = RC_MESH_NULL_IDX
	}
	n := 0
	// Add pa
	for i := 0; i < na-1; i++ {
		polys[tmp+n] = polys[pa+(ea+1+i)%na]
		n++
	}
	// Add pb
	for i := 0; i < nb-1; i++ {
		polys[tmp+n] = polys[pb+(eb+1+i)%nb]
		n++
	}
	arr := polys[tmp : tmp+nvp]
	copy(polys[pa:pa+nvp], arr)
}

func getPolyMergeValue(polys []int, pa int, pb int, verts []int, nvp int) []int {
	ea := -1
	eb := -1
	na := countPolyVerts(polys, pa, nvp)
	nb := countPolyVerts(polys, pb, nvp)
	// If the merged polygon would be too big, do not merge.
	if na+nb-2 > nvp {
		return []int{-1, ea, eb}
	}
	// Check if the polygons share an edge.
	for i := 0; i < na; i++ {
		va0 := polys[pa+i]
		va1 := polys[pa+(i+1)%na]
		if va0 > va1 {
			va0, va1 = va1, va0
		}
		for j := 0; j < nb; j++ {
			vb0 := polys[pb+j]
			vb1 := polys[pb+(j+1)%nb]
			if vb0 > vb1 {
				vb0, vb1 = vb1, vb0
			}
			if va0 == vb0 && va1 == vb1 {
				ea = i
				eb = j
				break
			}
		}
	}
	// No common edge, cannot merge.
	if ea == -1 || eb == -1 {
		return []int{-1, ea, eb}
	}
	// Check to see if the merged polygon would be convex.
	va, vb, vc := polys[pa+(ea+na-1)%na], polys[pa+ea], polys[pb+(eb+2)%nb]
	if !uleft(verts, va*3, vb*3, vc*3) {
		return []int{-1, ea, eb}
	}
	va = polys[pb+(eb+nb-1)%nb]
	vb = polys[pb+eb]
	vc = polys[pa+(ea+2)%na]
	if !uleft(verts, va*3, vb*3, vc*3) {
		return []int{-1, ea, eb}
	}
	va = polys[pa+ea]
	vb = polys[pa+(ea+1)%na]
	dx := verts[va*3+0] - verts[vb*3+0]
	dy := verts[va*3+2] - verts[vb*3+2]
	return []int{dx*dx + dy*dy, ea, eb}
}

func uleft(verts []int, a int, b int, c int) bool {
	return (verts[b+0]-verts[a+0])*(verts[c+2]-verts[a+2])-(verts[c+0]-verts[a+0])*(verts[b+2]-verts[a+2]) < 0
}
func countPolyVerts(p []int, j int, nvp int) int {
	for i := 0; i < nvp; i++ {
		if p[i+j] == RC_MESH_NULL_IDX {
			return i
		}
	}
	return nvp
}
func addVertex(x int, y int, z int, verts []int, firstVert []int, nextVert []int, nv int) []int {
	bucket := computeVertexHash(x, 0, z)
	i := firstVert[bucket]
	for i != -1 {
		v := i * 3
		if verts[v+0] == x && (int(math.Abs(float64(verts[v+1]-y))) <= 2) && verts[v+2] == z {
			return []int{i, nv}
		}
		i = nextVert[i] // next
	}
	// Could not find, create new.
	i = nv
	nv++
	v := i * 3
	verts[v+0] = x
	verts[v+1] = y
	verts[v+2] = z
	nextVert[i] = firstVert[bucket]
	firstVert[bucket] = i
	return []int{i, nv}
}

func computeVertexHash(x int, y int, z int) int {
	h1 := 0x8da6b343 // Large multiplicative constants;
	h2 := 0xd8163841 // here arbitrarily chosen primes
	h3 := 0xcb1ab31f
	n := h1*x + h2*y + h3*z
	return n & (VERTEX_BUCKET_COUNT - 1)
}

func triangulate(n int, verts []int, indices []int, tris []int) int {
	ntris := 0
	// The last bit of the index is used to indicate if the vertex can be removed.
	for i := 0; i < n; i++ {
		i1 := next(i, n)
		i2 := next(i1, n)
		if diagonal(i, i2, n, verts, indices) {
			indices[i1] |= 0x80000000
		}
	}
	for n > 3 {
		minLen := -1
		mini := -1
		for i := 0; i < n; i++ {
			i1 := next(i, n)
			if (indices[i1] & 0x80000000) != 0 {
				p0 := (indices[i] & 0x0fffffff) * 4
				p2 := (indices[next(i1, n)] & 0x0fffffff) * 4
				dx := verts[p2+0] - verts[p0+0]
				dy := verts[p2+2] - verts[p0+2]
				Len := dx*dx + dy*dy
				if minLen < 0 || Len < minLen {
					minLen = Len
					mini = i
				}
			}
		}
		if mini == -1 {
			// We might get here because the contour has overlapping segments, like this:
			//
			// A o-o=====o---o B
			// / |C D| \
			// o o o o
			// : : : :
			// We'll try to recover by loosing up the inCone test a bit so that a diagonal
			// like A-B or C-D can be found and we can continue.
			minLen = -1
			mini = -1
			for i := 0; i < n; i++ {
				i1 := next(i, n)
				i2 := next(i1, n)
				if diagonalLoose(i, i2, n, verts, indices) {
					p0 := (indices[i] & 0x0fffffff) * 4
					p2 := (indices[next(i2, n)] & 0x0fffffff) * 4
					dx := verts[p2+0] - verts[p0+0]
					dy := verts[p2+2] - verts[p0+2]
					Len := dx*dx + dy*dy
					if minLen < 0 || Len < minLen {
						minLen = Len
						mini = i
					}
				}
			}
			if mini == -1 {
				// The contour is messed up. This sometimes happens
				// if the contour simplification is too aggressive.
				return -ntris
			}
		}
		i := mini
		i1 := next(i, n)
		i2 := next(i1, n)
		tris[ntris*3] = indices[i] & 0x0fffffff
		tris[ntris*3+1] = indices[i1] & 0x0fffffff
		tris[ntris*3+2] = indices[i2] & 0x0fffffff
		ntris++
		// Removes P[i1] by copying P[i+1]...P[n-1] left one index.
		n--
		for k := i1; k < n; k++ {
			indices[k] = indices[k+1]
		}
		if i1 >= n {
			i1 = 0
		}
		i = prev(i1, n)
		// Update diagonal flags.
		if diagonal(prev(i, n), i1, n, verts, indices) {
			indices[i] |= 0x80000000
		} else {
			indices[i] &= 0x0fffffff
		}
		if diagonal(i, next(i1, n), n, verts, indices) {
			indices[i1] |= 0x80000000
		} else {
			indices[i1] &= 0x0fffffff
		}
	}
	// Append the remaining triangle.
	tris[ntris*3] = indices[0] & 0x0fffffff
	tris[ntris*3+1] = indices[1] & 0x0fffffff
	tris[ntris*3+2] = indices[2] & 0x0fffffff
	ntris++

	return ntris
}
func diagonalieLoose(i int, j int, n int, verts []int, indices []int) bool {
	d0 := (indices[i] & 0x0fffffff) * 4
	d1 := (indices[j] & 0x0fffffff) * 4
	// For each edge (k,k+1) of P
	for k := 0; k < n; k++ {
		k1 := next(k, n)
		// Skip edges incident to i or j
		if !((k == i) || (k1 == i) || (k == j) || (k1 == j)) {
			p0 := (indices[k] & 0x0fffffff) * 4
			p1 := (indices[k1] & 0x0fffffff) * 4
			if vequal(verts, d0, p0) || vequal(verts, d1, p0) || vequal(verts, d0, p1) || vequal(verts, d1, p1) {
				continue
			}
			if intersectProp(verts, d0, d1, p0, p1) {
				return false
			}
		}
	}
	return true
}
func diagonalLoose(i int, j int, n int, verts []int, indices []int) bool {
	return inConeLoose(i, j, n, verts, indices) && diagonalieLoose(i, j, n, verts, indices)
}

// Returns T iff (v_i, v_j) is a proper internal *or* external
// diagonal of P, *ignoring edges incident to v_i and v_j*.
func diagonalie(i int, j int, n int, verts []int, indices []int) bool {
	d0 := (indices[i] & 0x0fffffff) * 4
	d1 := (indices[j] & 0x0fffffff) * 4
	// For each edge (k,k+1) of P
	for k := 0; k < n; k++ {
		k1 := next(k, n)
		// Skip edges incident to i or j
		if !((k == i) || (k1 == i) || (k == j) || (k1 == j)) {
			p0 := (indices[k] & 0x0fffffff) * 4
			p1 := (indices[k1] & 0x0fffffff) * 4
			if vequal(verts, d0, p0) || vequal(verts, d1, p0) || vequal(verts, d0, p1) || vequal(verts, d1, p1) {
				continue
			}
			if intersect(verts, d0, d1, p0, p1) {
				return false
			}
		}
	}
	return true
}
func diagonal(i int, j int, n int, verts []int, indices []int) bool {
	return inCone5(i, j, n, verts, indices) && diagonalie(i, j, n, verts, indices)
}
func inConeLoose(i int, j int, n int, verts []int, indices []int) bool {
	pi := (indices[i] & 0x0fffffff) * 4
	pj := (indices[j] & 0x0fffffff) * 4
	pi1 := (indices[next(i, n)] & 0x0fffffff) * 4
	pin1 := (indices[prev(i, n)] & 0x0fffffff) * 4
	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	if leftOn(verts, pin1, pi, pi1) {
		return leftOn(verts, pi, pj, pin1) && leftOn(verts, pj, pi, pi1)
	}
	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	return !(leftOn(verts, pi, pj, pi1) && leftOn(verts, pj, pi, pin1))
}
func inCone5(i int, j int, n int, verts []int, indices []int) bool {
	pi := (indices[i] & 0x0fffffff) * 4
	pj := (indices[j] & 0x0fffffff) * 4
	pi1 := (indices[next(i, n)] & 0x0fffffff) * 4
	pin1 := (indices[prev(i, n)] & 0x0fffffff) * 4
	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	if leftOn(verts, pin1, pi, pi1) {
		return left(verts, pi, pj, pin1) && left(verts, pj, pi, pi1)
	}
	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	return !(leftOn(verts, pi, pj, pi1) && leftOn(verts, pj, pi, pin1))
}
func next(i, n int) int {
	if i+1 < n {
		return i + 1
	}
	return 0
}
func prev(i, n int) int {
	if i-1 >= 0 {
		return i - 1
	}
	return n - 1
}
func vequal(verts []int, a int, b int) bool {
	return verts[a+0] == verts[b+0] && verts[a+2] == verts[b+2]
}
func intersect(verts []int, a int, b int, c int, d int) bool {
	if intersectProp(verts, a, b, c, d) {
		return true
	} else if between(verts, a, b, c) || between(verts, a, b, d) || between(verts, c, d, a) || between(verts, c, d, b) {
		return true
	}
	return false
}
func intersectProp(verts []int, a int, b int, c int, d int) bool {
	// Eliminate improper cases.
	if collinear(verts, a, b, c) || collinear(verts, a, b, d) || collinear(verts, c, d, a) || collinear(verts, c, d, b) {
		return false
	}
	if (left(verts, a, b, c) != left(verts, a, b, d)) && (left(verts, c, d, a) != left(verts, c, d, b)) {
		return true
	}
	return false
}
func between(verts []int, a int, b int, c int) bool {
	if !collinear(verts, a, b, c) {
		return false
	}
	// If ab not vertical, check betweenness on x; else on y.
	if verts[a+0] != verts[b+0] {
		return ((verts[a+0] <= verts[c+0]) && (verts[c+0] <= verts[b+0]))
	}
	return ((verts[a+2] <= verts[c+2]) && (verts[c+2] <= verts[b+2])) || ((verts[a+2] >= verts[c+2]) && (verts[c+2] >= verts[b+2]))
}
func collinear(verts []int, a int, b int, c int) bool {
	return area2(verts, a, b, c) == 0
}
func area2(verts []int, a int, b int, c int) int {
	return (verts[b+0]-verts[a+0])*(verts[c+2]-verts[a+2]) - (verts[c+0]-verts[a+0])*(verts[b+2]-verts[a+2])
}
func left(verts []int, a int, b int, c int) bool {
	if area2(verts, a, b, c) < 0 {
		return true
	}
	return false
}
func leftOn(verts []int, a int, b int, c int) bool {
	return area2(verts, a, b, c) <= 0
}
