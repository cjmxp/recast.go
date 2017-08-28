package recast

import (
	"math"
	"strconv"
)

func BuildPolyMeshDetail(mesh *PolyMesh, chf *CompactHeightfield, sampleDist float32, sampleMaxError float32) *PolyMeshDetail {
	if mesh.nverts == 0 || mesh.npolys == 0 {
		return nil
	}
	dmesh := &PolyMeshDetail{}
	nvp := mesh.nvp
	cs := mesh.cs
	ch := mesh.ch
	orig := mesh.bmin
	borderSize := mesh.borderSize
	heightSearchRadius := int(math.Max(1, math.Ceil(float64(mesh.maxEdgeError))))
	tris := []int{}
	verts := make([]float32, 256*3)
	hp := &HeightPatch{}
	nPolyVerts := 0
	maxhw, maxhh := 0, 0
	bounds := make([]int, mesh.npolys*4)
	poly := make([]float32, nvp*3)
	// Find max size for a polygon area.
	for i := 0; i < mesh.npolys; i++ {
		p := i * nvp * 2
		bounds[i*4+0] = chf.width
		bounds[i*4+1] = 0
		bounds[i*4+2] = chf.height
		bounds[i*4+3] = 0
		for j := 0; j < nvp; j++ {
			if mesh.polys[p+j] == RC_MESH_NULL_IDX {
				break
			}
			v := mesh.polys[p+j] * 3
			bounds[i*4+0] = int(math.Min(float64(bounds[i*4+0]), float64(mesh.verts[v+0])))
			bounds[i*4+1] = int(math.Max(float64(bounds[i*4+1]), float64(mesh.verts[v+0])))
			bounds[i*4+2] = int(math.Min(float64(bounds[i*4+2]), float64(mesh.verts[v+2])))
			bounds[i*4+3] = int(math.Max(float64(bounds[i*4+3]), float64(mesh.verts[v+2])))
			nPolyVerts++
		}
		bounds[i*4+0] = int(math.Max(0, float64(bounds[i*4+0]-1)))
		bounds[i*4+1] = int(math.Min(float64(chf.width), float64(bounds[i*4+1]+1)))
		bounds[i*4+2] = int(math.Max(0, float64(bounds[i*4+2]-1)))
		bounds[i*4+3] = int(math.Min(float64(chf.height), float64(bounds[i*4+3]+1)))
		if bounds[i*4+0] >= bounds[i*4+1] || bounds[i*4+2] >= bounds[i*4+3] {
			continue
		}
		maxhw = int(math.Max(float64(maxhw), float64(bounds[i*4+1]-bounds[i*4+0])))
		maxhh = int(math.Max(float64(maxhh), float64(bounds[i*4+3]-bounds[i*4+2])))
	}
	hp.data = make([]int, maxhw*maxhh)
	dmesh.nmeshes = mesh.npolys
	dmesh.nverts = 0
	dmesh.ntris = 0
	dmesh.meshes = make([]int, dmesh.nmeshes*4)
	vcap := nPolyVerts + nPolyVerts/2
	tcap := vcap * 2
	dmesh.nverts = 0
	dmesh.verts = make([]float32, vcap*3)
	dmesh.ntris = 0
	dmesh.tris = make([]int, tcap*4)
	for i := 0; i < mesh.npolys; i++ {
		p := i * nvp * 2
		// Store polygon vertices for processing.
		npoly := 0
		for j := 0; j < nvp; j++ {
			if mesh.polys[p+j] == RC_MESH_NULL_IDX {
				break
			}
			v := mesh.polys[p+j] * 3
			poly[j*3+0] = float32(mesh.verts[v+0]) * cs
			poly[j*3+1] = float32(mesh.verts[v+1]) * ch
			poly[j*3+2] = float32(mesh.verts[v+2]) * cs
			npoly++
		}
		// Get the height data from the area of the polygon.
		hp.xmin = bounds[i*4+0]
		hp.ymin = bounds[i*4+2]
		hp.width = bounds[i*4+1] - bounds[i*4+0]
		hp.height = bounds[i*4+3] - bounds[i*4+2]
		getHeightData(chf, mesh.polys, p, npoly, mesh.verts, borderSize, hp, mesh.regs[i])
		// Build detail mesh.
		nverts := buildPolyDetail(poly, npoly, sampleDist, sampleMaxError, heightSearchRadius, chf, hp, verts, &tris)
		// Move detail verts to world space.
		for j := 0; j < nverts; j++ {
			verts[j*3+0] += orig[0]
			verts[j*3+1] += orig[1] + chf.ch // Is this offset necessary?
			verts[j*3+2] += orig[2]
		}
		// Offset poly too, will be used to flag checking.
		for j := 0; j < npoly; j++ {
			poly[j*3+0] += orig[0]
			poly[j*3+1] += orig[1]
			poly[j*3+2] += orig[2]
		}
		// Store detail submesh.
		ntris := len(tris) / 4
		dmesh.meshes[i*4+0] = dmesh.nverts
		dmesh.meshes[i*4+1] = nverts
		dmesh.meshes[i*4+2] = dmesh.ntris
		dmesh.meshes[i*4+3] = ntris
		// Store vertices, allocate more memory if necessary.
		if dmesh.nverts+nverts > vcap {
			for dmesh.nverts+nverts > vcap {
				vcap += 256
			}
			newv := make([]float32, vcap*3)
			if dmesh.nverts != 0 {
				copy(newv, dmesh.verts[:3*dmesh.nverts])
			}
			dmesh.verts = newv
		}
		for j := 0; j < nverts; j++ {
			dmesh.verts[dmesh.nverts*3+0] = verts[j*3+0]
			dmesh.verts[dmesh.nverts*3+1] = verts[j*3+1]
			dmesh.verts[dmesh.nverts*3+2] = verts[j*3+2]
			dmesh.nverts++
		}
		// Store triangles, allocate more memory if necessary.
		if dmesh.ntris+ntris > tcap {
			for dmesh.ntris+ntris > tcap {
				tcap += 256
			}
			newt := make([]int, tcap*4)
			if dmesh.ntris != 0 {
				copy(newt, dmesh.tris[:4*dmesh.ntris])
			}
			dmesh.tris = newt
		}
		for j := 0; j < ntris; j++ {
			t := j * 4
			dmesh.tris[dmesh.ntris*4+0] = tris[t+0]
			dmesh.tris[dmesh.ntris*4+1] = tris[t+1]
			dmesh.tris[dmesh.ntris*4+2] = tris[t+2]
			dmesh.tris[dmesh.ntris*4+3] = getTriFlags(verts, tris[t+0]*3, tris[t+1]*3, tris[t+2]*3, poly, npoly)
			dmesh.ntris++
		}
	}
	return dmesh
}

func getTriFlags(verts []float32, va int, vb int, vc int, vpoly []float32, npoly int) int {
	flags := 0
	flags |= getEdgeFlags(verts, va, vb, vpoly, npoly) << 0
	flags |= getEdgeFlags(verts, vb, vc, vpoly, npoly) << 2
	flags |= getEdgeFlags(verts, vc, va, vpoly, npoly) << 4
	return flags
}

func getEdgeFlags(verts []float32, va int, vb int, vpoly []float32, npoly int) int {
	// Return true if edge (va,vb) is part of the polygon.
	thrSqr := float32(0.001 * 0.001)
	for i, j := 0, npoly-1; i < npoly; j, i = i, i+1 {
		if distancePtSeg2d(verts, va, vpoly, j*3, i*3) < thrSqr && distancePtSeg2d(verts, vb, vpoly, j*3, i*3) < thrSqr {
			return 1
		}
	}
	return 0
}

func buildPolyDetail(in []float32, nin int, sampleDist float32, sampleMaxError float32, heightSearchRadius int, chf *CompactHeightfield, hp *HeightPatch, verts []float32, tris *[]int) int {
	samples := []int{}
	nverts := 0
	edge := make([]float32, (MAX_VERTS_PER_EDGE+1)*3)
	hull := make([]int, MAX_VERTS)
	nhull := 0
	nverts = nin
	for i := 0; i < nin; i++ {
		copy3(verts, i*3, in, i*3)
	}
	*tris = []int{}
	cs := chf.cs
	ics := 1.0 / cs
	// Calculate minimum extents of the polygon based on input data.
	minExtent := polyMinExtent(verts, nverts)
	// Tessellate outlines.
	// This is done in separate pass in order to ensure
	// seamless height values across the ply boundaries.
	if sampleDist > 0 {
		for i, j := 0, nin-1; i < nin; j, i = i, i+1 {
			vj := j * 3
			vi := i * 3
			swapped := false
			// Make sure the segments are always handled in same order
			// using lexological sort or else there will be seams.
			if float32(math.Abs(float64(in[vj+0]-in[vi+0]))) < float32(1.0E-6) {
				if in[vj+2] > in[vi+2] {
					vi, vj = vj, vi
					swapped = true
				}
			} else {
				if in[vj+0] > in[vi+0] {
					vi, vj = vj, vi
					swapped = true
				}
			}
			// Create samples along the edge.
			dx := in[vi+0] - in[vj+0]
			dy := in[vi+1] - in[vj+1]
			dz := in[vi+2] - in[vj+2]
			d := float32(math.Sqrt(float64(dx*dx + dz*dz)))
			nn := 1 + int(math.Floor(float64(d/sampleDist)))
			if nn >= MAX_VERTS_PER_EDGE {
				nn = MAX_VERTS_PER_EDGE - 1
			}
			if nverts+nn >= MAX_VERTS {
				nn = MAX_VERTS - 1 - nverts
			}
			for k := 0; k <= nn; k++ {
				u := float32(k) / float32(nn)
				pos := k * 3
				edge[pos+0] = in[vj+0] + dx*u
				edge[pos+1] = in[vj+1] + dy*u
				edge[pos+2] = in[vj+2] + dz*u
				edge[pos+1] = float32(getHeight(edge[pos+0], edge[pos+1], edge[pos+2], cs, ics, chf.ch, heightSearchRadius, hp)) * chf.ch
			}
			// Simplify samples.
			idx := make([]int, MAX_VERTS_PER_EDGE)
			idx[0] = 0
			idx[1] = nn
			nidx := 2
			for k := 0; k < nidx-1; {
				a := idx[k]
				b := idx[k+1]
				va := a * 3
				vb := b * 3
				// Find maximum deviation along the segment.
				maxd := float32(0)
				maxi := -1
				for m := a + 1; m < b; m++ {
					dev := distancePtSeg4(edge, m*3, va, vb)
					if dev > maxd {
						maxd = dev
						maxi = m
					}
				}
				// If the max deviation is larger than accepted error,
				// add new point, else continue to next segment.
				if maxi != -1 && maxd > sampleMaxError*sampleMaxError {
					for m := nidx; m > k; m-- {
						idx[m] = idx[m-1]
					}
					idx[k+1] = maxi
					nidx++
				} else {
					k++
				}
			}
			hull[nhull] = j
			nhull++
			// Add new vertices.
			if swapped {
				for k := nidx - 2; k > 0; k-- {
					copy3(verts, nverts*3, edge, idx[k]*3)
					hull[nhull] = nverts
					nhull++
					nverts++
				}
			} else {
				for k := 1; k < nidx-1; k++ {
					copy3(verts, nverts*3, edge, idx[k]*3)
					hull[nhull] = nverts
					nhull++
					nverts++
				}
			}
		}
	}
	// If the polygon minimum extent is small (sliver or small triangle), do not try to add internal points.
	if minExtent < sampleDist*2 {
		triangulateHull(nverts, verts, nhull, hull, tris)
		return nverts
	}
	// Tessellate the base mesh.
	// We're using the triangulateHull instead of delaunayHull as it tends to
	// create a bit better triangulation for long thin triangles when there
	// are no internal points.
	triangulateHull(nverts, verts, nhull, hull, tris)
	if len(*tris) == 0 {
		// Could not triangulate the poly, make sure there is some valid data there.
		panic("buildPolyDetail: Could not triangulate polygon (" + strconv.Itoa(nverts) + ") verts).")
	}
	if sampleDist > 0 {
		// Create sample locations in a grid.
		bmin := make([]float32, 3)
		bmax := make([]float32, 3)
		copy3(bmin, 0, in, 0)
		copy3(bmax, 0, in, 0)
		for i := 1; i < nin; i++ {
			min(bmin, in, i*3)
			max(bmax, in, i*3)
		}
		x0 := int(math.Floor(float64(bmin[0] / sampleDist)))
		x1 := int(math.Ceil(float64(bmax[0] / sampleDist)))
		z0 := int(math.Floor(float64(bmin[2] / sampleDist)))
		z1 := int(math.Ceil(float64(bmax[2] / sampleDist)))
		samples = []int{}
		for z := z0; z < z1; z++ {
			for x := x0; x < x1; x++ {
				pt := make([]float32, 3)
				pt[0] = float32(x) * sampleDist
				pt[1] = (bmax[1] + bmin[1]) * 0.5
				pt[2] = float32(z) * sampleDist
				// Make sure the samples are not too close to the edges.
				if distToPoly(nin, in, pt) > -sampleDist/2 {
					continue
				}
				samples = append(samples, x, getHeight(pt[0], pt[1], pt[2], cs, ics, chf.ch, heightSearchRadius, hp), z, 0)
			}
		}
		// Add the samples starting from the one that has the most
		// error. The procedure stops when all samples are added
		// or when the max error is within treshold.
		nsamples := len(samples) / 4
		for iter := 0; iter < nsamples; iter++ {
			if nverts >= MAX_VERTS {
				break
			}
			// Find sample with most error.
			bestpt := make([]float32, 3)
			bestd := float32(0)
			besti := -1
			for i := 0; i < nsamples; i++ {
				s := i * 4
				if samples[s+3] != 0 {
					continue // skip added.
				}
				pt := make([]float32, 3)
				// The sample location is jittered to get rid of some bad triangulations
				// which are cause by symmetrical data from the grid structure.
				pt[0] = float32(samples[s+0])*sampleDist + getJitterX(i)*cs*0.1
				pt[1] = float32(samples[s+1]) * chf.ch
				pt[2] = float32(samples[s+2])*sampleDist + getJitterY(i)*cs*0.1
				d := distToTriMesh(pt, verts, nverts, tris, len(*tris)/4)
				if d < 0 {
					continue // did not hit the mesh.
				}
				if d > bestd {
					bestd = d
					besti = i
					bestpt = pt
				}
			}
			// If the max error is within accepted threshold, stop tesselating.
			if bestd <= sampleMaxError || besti == -1 {
				break
			}
			// Mark sample as added.
			samples[besti*4+3] = 1
			// Add the new sample point.
			copy3(verts, nverts*3, bestpt, 0)
			nverts++
			// Create new triangulation.
			// TODO: Incremental add instead of full rebuild.
			delaunayHull(nverts, verts, nhull, hull, tris)
		}
	}
	ntris := len(*tris) / 4
	if ntris > MAX_TRIS {
		*tris = (*tris)[:MAX_TRIS*4]
		panic("rcBuildPolyMeshDetail: Shrinking triangle count from " + strconv.Itoa(ntris) + " to max " + strconv.Itoa(MAX_TRIS))
	}
	return nverts
}

func delaunayHull(npts int, pts []float32, nhull int, hull []int, tris *[]int) {
	nfaces := 0
	maxEdges := npts * 10
	edges := []int{}
	for i, j := 0, nhull-1; i < nhull; j, i = i, i+1 {
		addEdge(&edges, maxEdges, hull[j], hull[i], EV_HULL, EV_UNDEF)
	}
	currentEdge := 0
	for currentEdge < len(edges)/4 {
		if edges[currentEdge*4+2] == EV_UNDEF {
			nfaces = completeFacet(pts, npts, &edges, maxEdges, nfaces, currentEdge)
		}
		if edges[currentEdge*4+3] == EV_UNDEF {
			nfaces = completeFacet(pts, npts, &edges, maxEdges, nfaces, currentEdge)
		}
		currentEdge++
	}
	// Create tris
	*tris = []int{}
	for i := 0; i < nfaces*4; i++ {
		*tris = append(*tris, -1)
	}
	for i := 0; i < len(edges)/4; i++ {
		e := i * 4
		if edges[e+3] >= 0 {
			// Left face
			t := edges[e+3] * 4
			if (*tris)[t+0] == -1 {
				(*tris)[t+0] = edges[e+0]
				(*tris)[t+1] = edges[e+1]
			} else if (*tris)[t+0] == edges[e+1] {
				(*tris)[t+2] = edges[e+0]
			} else if (*tris)[t+1] == edges[e+0] {
				(*tris)[t+2] = edges[e+1]
			}
		}
		if edges[e+2] >= 0 {
			// Right
			t := edges[e+2] * 4
			if (*tris)[t+0] == -1 {
				(*tris)[t+0] = edges[e+1]
				(*tris)[t+1] = edges[e+0]
			} else if (*tris)[t+0] == edges[e+0] {
				(*tris)[t+2] = edges[e+1]
			} else if (*tris)[t+1] == edges[e+1] {
				(*tris)[t+2] = edges[e+0]
			}
		}
	}
	for i := 0; i < len(*tris)/4; i++ {
		t := i * 4
		if (*tris)[t+0] == -1 || (*tris)[t+1] == -1 || (*tris)[t+2] == -1 {
			println("Dangling! ", (*tris)[t], (*tris)[t+1], (*tris)[t+2])
			//ctx.log(RC_LOG_WARNING, "delaunayHull: Removing dangling face %d [%d,%d,%d].", i, t[0],t[1],t[2]);
			(*tris)[t+0] = (*tris)[len(*tris)-4]
			(*tris)[t+1] = (*tris)[len(*tris)-3]
			(*tris)[t+2] = (*tris)[len(*tris)-2]
			(*tris)[t+3] = (*tris)[len(*tris)-1]
			*tris = (*tris)[:len(*tris)-4]
			i--
		}
	}
}
func completeFacet(pts []float32, npts int, edges *[]int, maxEdges int, nfaces int, e int) int {
	EPS := float32(1e-5)
	edge := e * 4
	// Cache s and t.
	s, t := 0, 0
	if (*edges)[edge+2] == EV_UNDEF {
		s = (*edges)[edge+0]
		t = (*edges)[edge+1]
	} else if (*edges)[edge+3] == EV_UNDEF {
		s = (*edges)[edge+1]
		t = (*edges)[edge+0]
	} else {
		// Edge already completed.
		return nfaces
	}
	// Find best point on left of edge.
	pt := npts
	c := make([]float32, 3)
	r := float32(-1)
	for u := 0; u < npts; u++ {
		if u == s || u == t {
			continue
		}
		if vcross2(pts, s*3, t*3, u*3) > EPS {
			if r < 0 {
				pt = u
				circumCircle(pts, s*3, t*3, u*3, c, &r)
				continue
			}
			d := vdist3(c, pts, u*3)
			tol := float32(0.001)
			if d > r*(1+tol) {
				// Outside current circumcircle, skip.
				continue
			} else if d < r*(1-tol) {
				// Inside safe circumcircle, update circle.
				pt = u
				circumCircle(pts, s*3, t*3, u*3, c, &r)
			} else {
				// Inside epsilon circum circle, do extra tests to make sure the edge is valid.
				// s-u and t-u cannot overlap with s-pt nor t-pt if they exists.
				if overlapEdges(pts, edges, s, u) {
					continue
				}
				if overlapEdges(pts, edges, t, u) {
					continue
				}
				// Edge is valid.
				pt = u
				circumCircle(pts, s*3, t*3, u*3, c, &r)
			}
		}
	}
	// Add new triangle or update edge info if s-t is on hull.
	if pt < npts {
		// Update face information of edge being completed.
		updateLeftFace(edges, e*4, s, t, nfaces)
		// Add new edge or update face info of old edge.
		e = findEdge(edges, pt, s)
		if e == EV_UNDEF {
			addEdge(edges, maxEdges, pt, s, nfaces, EV_UNDEF)
		} else {
			updateLeftFace(edges, e*4, pt, s, nfaces)
		}
		// Add new edge or update face info of old edge.
		e = findEdge(edges, t, pt)
		if e == EV_UNDEF {
			addEdge(edges, maxEdges, t, pt, nfaces, EV_UNDEF)
		} else {
			updateLeftFace(edges, e*4, t, pt, nfaces)
		}
		nfaces++
	} else {
		updateLeftFace(edges, e*4, s, t, EV_HULL)
	}
	return nfaces
}

func updateLeftFace(edges *[]int, e int, s int, t int, f int) {
	if (*edges)[e+0] == s && (*edges)[e+1] == t && (*edges)[e+2] == EV_UNDEF {
		(*edges)[e+2] = f
	} else if (*edges)[e+1] == s && (*edges)[e+0] == t && (*edges)[e+3] == EV_UNDEF {
		(*edges)[e+3] = f
	}
}
func overlapEdges(pts []float32, edges *[]int, s1 int, t1 int) bool {
	for i := 0; i < len(*edges)/4; i++ {
		s0 := (*edges)[i*4+0]
		t0 := (*edges)[i*4+1]
		// Same or connected edges do not overlap.
		if s0 == s1 || s0 == t1 || t0 == s1 || t0 == t1 {
			continue
		}
		if overlapSegSeg2d(pts, s0*3, t0*3, s1*3, t1*3) {
			return true
		}
	}
	return false
}
func overlapSegSeg2d(verts []float32, a int, b int, c int, d int) bool {
	a1 := vcross2(verts, a, b, d)
	a2 := vcross2(verts, a, b, c)
	if a1*a2 < 0.0 {
		a3 := vcross2(verts, c, d, a)
		a4 := a3 + a2 - a1
		if a3*a4 < 0.0 {
			return true
		}

	}
	return false
}
func circumCircle(verts []float32, p1 int, p2 int, p3 int, c []float32, r *float32) bool {
	EPS := float32(1e-6)
	v1 := make([]float32, 3)
	v2 := make([]float32, 3)
	v3 := make([]float32, 3)
	sub(v2, verts, p2, p1)
	sub(v3, verts, p3, p1)
	cp := vcross3(v1, v2, v3)
	if float32(math.Abs(float64(cp))) > EPS {
		v1Sq := vdot2(v1, v1)
		v2Sq := vdot2(v2, v2)
		v3Sq := vdot2(v3, v3)
		c[0] = (v1Sq*(v2[2]-v3[2]) + v2Sq*(v3[2]-v1[2]) + v3Sq*(v1[2]-v2[2])) / (2 * cp)
		c[1] = 0
		c[2] = (v1Sq*(v3[0]-v2[0]) + v2Sq*(v1[0]-v3[0]) + v3Sq*(v2[0]-v1[0])) / (2 * cp)
		*r = vdist1(c, v1)
		add(c, c, verts, p1)
		return true
	}
	copy3(c, 0, verts, p1)
	*r = float32(0.0)
	return false
}
func vcross3(p1 []float32, p2 []float32, p3 []float32) float32 {
	u1 := p2[0] - p1[0]
	v1 := p2[2] - p1[2]
	u2 := p3[0] - p1[0]
	v2 := p3[2] - p1[2]
	return u1*v2 - v1*u2
}
func vcross2(verts []float32, p1 int, p2 int, p3 int) float32 {
	u1 := verts[p2+0] - verts[p1+0]
	v1 := verts[p2+2] - verts[p1+2]
	u2 := verts[p3+0] - verts[p1+0]
	v2 := verts[p3+2] - verts[p1+2]
	return u1*v2 - v1*u2
}

func addEdge(edges *[]int, maxEdges int, s int, t int, l int, r int) {
	if len(*edges)/4 >= maxEdges {
		panic("addEdge: Too many edges (" + strconv.Itoa(len(*edges)/4) + "/" + strconv.Itoa(maxEdges) + ").")
	}
	// Add edge if not already in the triangulation.
	e := findEdge(edges, s, t)
	if e == EV_UNDEF {
		*edges = append(*edges, s, t, l, r)
	}
}

func findEdge(edges *[]int, s int, t int) int {
	for i := 0; i < len(*edges)/4; i++ {
		e := i * 4
		if ((*edges)[e+0] == s && (*edges)[e+1] == t) || ((*edges)[e+0] == t && (*edges)[e+1] == s) {
			return i
		}
	}
	return EV_UNDEF
}

func distToTriMesh(p []float32, verts []float32, nverts int, tris *[]int, ntris int) float32 {
	dmin := FLOAT_MAX_VALUE
	for i := 0; i < ntris; i++ {
		va := (*tris)[i*4+0] * 3
		vb := (*tris)[i*4+1] * 3
		vc := (*tris)[i*4+2] * 3
		d := distPtTri(p, verts, va, vb, vc)
		if d < dmin {
			dmin = d
		}
	}
	if dmin == FLOAT_MAX_VALUE {
		return -1
	}
	return dmin
}
func distPtTri(p []float32, verts []float32, a int, b int, c int) float32 {
	v0 := make([]float32, 3)
	v1 := make([]float32, 3)
	v2 := make([]float32, 3)
	sub(v0, verts, c, a)
	sub(v1, verts, b, a)
	sub3(v2, p, verts, a)
	dot00 := vdot2(v0, v0)
	dot01 := vdot2(v0, v1)
	dot02 := vdot2(v0, v2)
	dot11 := vdot2(v1, v1)
	dot12 := vdot2(v1, v2)
	// Compute barycentric coordinates
	invDenom := 1.0 / (dot00*dot11 - dot01*dot01)
	u := (dot11*dot02 - dot01*dot12) * invDenom
	v := (dot00*dot12 - dot01*dot02) * invDenom
	// If point lies inside the triangle, return interpolated y-coord.
	EPS := float32(1e-4)
	if u >= -EPS && v >= -EPS && (u+v) <= 1+EPS {
		y := verts[a+1] + v0[1]*u + v1[1]*v
		return float32(math.Abs(float64(y - p[1])))
	}
	return FLOAT_MAX_VALUE
}

func vdot2(a []float32, b []float32) float32 {
	return a[0]*b[0] + a[2]*b[2]
}

func getJitterX(i int) float32 {
	return (float32(((i * 0x8da6b343) & 0xffff)) / 65535.0 * 2.0) - 1.0
}
func getJitterY(i int) float32 {
	return (float32(((i * 0xd8163841) & 0xffff)) / 65535.0 * 2.0) - 1.0
}
func distToPoly(nvert int, verts []float32, p []float32) float32 {
	dmin := FLOAT_MAX_VALUE
	c := false
	for i, j := 0, nvert-1; i < nvert; j, i = i, i+1 {
		vi := i * 3
		vj := j * 3
		if ((verts[vi+2] > p[2]) != (verts[vj+2] > p[2])) && (p[0] < (verts[vj+0]-verts[vi+0])*(p[2]-verts[vi+2])/(verts[vj+2]-verts[vi+2])+verts[vi+0]) {
			c = !c
		}
		dmin = float32(math.Min(float64(dmin), float64(distancePtSeg2d(p, 0, verts, vj, vi))))
	}
	if c {
		return -dmin
	}
	return dmin
}

func triangulateHull(nverts int, verts []float32, nhull int, hull []int, tris *[]int) {
	start, left, right := 0, 1, nhull-1
	// Start from an ear with shortest perimeter.
	// This tends to favor well formed triangles as starting point.
	dmin := float32(0)
	for i := 0; i < nhull; i++ {
		pi := prev(i, nhull)
		ni := next(i, nhull)
		pv := hull[pi] * 3
		cv := hull[i] * 3
		nv := hull[ni] * 3
		d := vdist2(verts, pv, cv) + vdist2(verts, cv, nv) + vdist2(verts, nv, pv)
		if d < dmin {
			start = i
			left = ni
			right = pi
			dmin = d
		}
	}
	// Add first triangle
	*tris = append(*tris, hull[start], hull[left], hull[right], 0)
	// Triangulate the polygon by moving left or right,
	// depending on which triangle has shorter perimeter.
	// This heuristic was chose emprically, since it seems
	// handle tesselated straight edges well.
	for next(left, nhull) != right {
		// Check to see if se should advance left or right.
		nleft := next(left, nhull)
		nright := prev(right, nhull)
		cvleft := hull[left] * 3
		nvleft := hull[nleft] * 3
		cvright := hull[right] * 3
		nvright := hull[nright] * 3
		dleft := vdist2(verts, cvleft, nvleft) + vdist2(verts, nvleft, cvright)
		dright := vdist2(verts, cvright, nvright) + vdist2(verts, cvleft, nvright)
		if dleft < dright {
			*tris = append(*tris, hull[left], hull[nleft], hull[right], 0)
			left = nleft
		} else {
			*tris = append(*tris, hull[left], hull[nright], hull[right], 0)
			right = nright
		}
	}
}
func vdist1(p []float32, q []float32) float32 {
	return float32(math.Sqrt(float64(vdistSq1(p, q))))
}

func vdist2(verts []float32, p int, q int) float32 {
	return float32(math.Sqrt(float64(vdistSq2(verts, p, q))))
}
func vdist3(p []float32, verts []float32, q int) float32 {
	return float32(math.Sqrt(float64(vdistSq3(p, verts, q))))
}
func vdistSq1(p []float32, q []float32) float32 {
	dx := q[0] - p[0]
	dy := q[2] - p[2]
	return dx*dx + dy*dy
}
func vdistSq2(verts []float32, p int, q int) float32 {
	dx := verts[q+0] - verts[p+0]
	dy := verts[q+2] - verts[p+2]
	return dx*dx + dy*dy
}
func vdistSq3(p []float32, verts []float32, q int) float32 {
	dx := verts[q+0] - p[0]
	dy := verts[q+2] - p[2]
	return dx*dx + dy*dy
}

func distancePtSeg4(verts []float32, pt int, p int, q int) float32 {
	pqx := verts[q+0] - verts[p+0]
	pqy := verts[q+1] - verts[p+1]
	pqz := verts[q+2] - verts[p+2]
	dx := verts[pt+0] - verts[p+0]
	dy := verts[pt+1] - verts[p+1]
	dz := verts[pt+2] - verts[p+2]
	d := pqx*pqx + pqy*pqy + pqz*pqz
	t := pqx*dx + pqy*dy + pqz*dz
	if d > 0 {
		t /= d
	}

	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	dx = verts[p+0] + t*pqx - verts[pt+0]
	dy = verts[p+1] + t*pqy - verts[pt+1]
	dz = verts[p+2] + t*pqz - verts[pt+2]
	return dx*dx + dy*dy + dz*dz
}
func getHeight(fx float32, fy float32, fz float32, cs float32, ics float32, ch float32, radius int, hp *HeightPatch) int {
	ix := int(math.Floor(float64(fx*ics + 0.01)))
	iz := int(math.Floor(float64(fz*ics + 0.01)))
	ix = clamp_i(ix-hp.xmin, 0, hp.width-1)
	iz = clamp_i(iz-hp.ymin, 0, hp.height-1)
	h := hp.data[ix+iz*hp.width]
	if h == RC_UNSET_HEIGHT {
		// Special case when data might be bad.
		// Walk adjacent cells in a spiral up to 'radius', and look
		// for a pixel which has a valid height.
		x, z, dx, dz := 1, 0, 1, 0
		maxSize := radius*2 + 1
		maxIter := maxSize*maxSize - 1
		nextRingIterStart := 8
		nextRingIters := 16
		dmin := FLOAT_MAX_VALUE
		for i := 0; i < maxIter; i++ {
			nx := ix + x
			nz := iz + z
			if nx >= 0 && nz >= 0 && nx < hp.width && nz < hp.height {
				nh := hp.data[nx+nz*hp.width]
				if nh != RC_UNSET_HEIGHT {
					d := float32(math.Abs(float64(float32(nh)*ch - fy)))
					if d < dmin {
						h = nh
						dmin = d
					}
				}
			}
			// We are searching in a grid which looks approximately like this:
			//  __________
			// |2 ______ 2|
			// | |1 __ 1| |
			// | | |__| | |
			// | |______| |
			// |__________|
			// We want to find the best height as close to the center cell as possible. This means that
			// if we find a height in one of the neighbor cells to the center, we don't want to
			// expand further out than the 8 neighbors - we want to limit our search to the closest
			// of these "rings", but the best height in the ring.
			// For example, the center is just 1 cell. We checked that at the entrance to the function.
			// The next "ring" contains 8 cells (marked 1 above). Those are all the neighbors to the center cell.
			// The next one again contains 16 cells (marked 2). In general each ring has 8 additional cells, which
			// can be thought of as adding 2 cells around the "center" of each side when we expand the ring.
			// Here we detect if we are about to enter the next ring, and if we are and we have found
			// a height, we abort the search.
			if i+1 == nextRingIterStart {
				if h != RC_UNSET_HEIGHT {
					break
				}
				nextRingIterStart += nextRingIters
				nextRingIters += 8
			}
			if (x == z) || ((x < 0) && (x == -z)) || ((x > 0) && (x == 1-z)) {
				dx, dz = -dz, dx
			}
			x += dx
			z += dz
		}
	}
	return h
}

func polyMinExtent(verts []float32, nverts int) float32 {
	minDist := FLOAT_MAX_VALUE
	for i := 0; i < nverts; i++ {
		ni := (i + 1) % nverts
		p1 := i * 3
		p2 := ni * 3
		maxEdgeDist := float32(0)
		for j := 0; j < nverts; j++ {
			if j == i || j == ni {
				continue
			}
			d := distancePtSeg2d(verts, j*3, verts, p1, p2)
			maxEdgeDist = float32(math.Max(float64(maxEdgeDist), float64(d)))
		}
		minDist = float32(math.Min(float64(minDist), float64(maxEdgeDist)))
	}
	return float32(math.Sqrt(float64(minDist)))
}

func distancePtSeg2d(verts []float32, pt int, poly []float32, p int, q int) float32 {
	pqx := poly[q+0] - poly[p+0]
	pqz := poly[q+2] - poly[p+2]
	dx := verts[pt+0] - poly[p+0]
	dz := verts[pt+2] - poly[p+2]
	d := pqx*pqx + pqz*pqz
	t := pqx*dx + pqz*dz
	if d > 0 {
		t /= d
	}
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	dx = poly[p+0] + t*pqx - verts[pt+0]
	dz = poly[p+2] + t*pqz - verts[pt+2]
	return dx*dx + dz*dz
}

func getHeightData(chf *CompactHeightfield, meshpolys []int, poly int, npoly int, verts []int, bs int, hp *HeightPatch, region int) {
	// Note: Reads to the compact heightfield are offset by border size (bs)
	// since border size offset is already removed from the polymesh vertices.
	queue := []int{}
	for i := 0; i < hp.width*hp.height; i++ {
		hp.data[i] = RC_UNSET_HEIGHT
	}
	empty := true
	// We cannot sample from this poly if it was created from polys
	// of different regions. If it was then it could potentially be overlapping
	// with polys of that region and the heights sampled here could be wrong.
	if region != RC_MULTIPLE_REGS {
		// Copy the height from the same region, and mark region borders
		// as seed points to fill the rest.
		for hy := 0; hy < hp.height; hy++ {
			y := hp.ymin + hy + bs
			for hx := 0; hx < hp.width; hx++ {
				x := hp.xmin + hx + bs
				c := &chf.cells[x+y*chf.width]
				for i, ni := c.index, c.index+c.count; i < ni; i++ {
					s := &chf.spans[i]
					if s.reg == region {
						// Store height
						hp.data[hx+hy*hp.width] = s.y
						empty = false
						// If any of the neighbours is not in same region,
						// add the current location as flood fill start
						border := false
						for dir := 0; dir < 4; dir++ {
							if GetCon(s, dir) != RC_NOT_CONNECTED {
								ax := x + GetDirOffsetX(dir)
								ay := y + GetDirOffsetY(dir)
								ai := chf.cells[ax+ay*chf.width].index + GetCon(s, dir)
								as := &chf.spans[ai]
								if as.reg != region {
									border = true
									break
								}
							}
						}
						if border {
							push3(&queue, x, y, i)
						}
						break
					}
				}
			}
		}
	}
	// if the polygon does not contain any points from the current region (rare, but happens)
	// or if it could potentially be overlapping polygons of the same region,
	// then use the center as the seed point.
	if empty {
		seedArrayWithPolyCenter(chf, meshpolys, poly, npoly, verts, bs, hp, &queue)
	}
	head := 0
	// We assume the seed is centered in the polygon, so a BFS to collect
	// height data will ensure we do not move onto overlapping polygons and
	// sample wrong heights.
	for head*3 < len(queue) {
		cx := queue[head*3+0]
		cy := queue[head*3+1]
		ci := queue[head*3+2]
		head++
		if head >= RETRACT_SIZE {
			head = 0
			queue = queue[RETRACT_SIZE*3:]
		}
		cs := &chf.spans[ci]
		for dir := 0; dir < 4; dir++ {
			if GetCon(cs, dir) == RC_NOT_CONNECTED {
				continue
			}
			ax := cx + GetDirOffsetX(dir)
			ay := cy + GetDirOffsetY(dir)
			hx := ax - hp.xmin - bs
			hy := ay - hp.ymin - bs
			if hx < 0 || hx >= hp.width || hy < 0 || hy >= hp.height {
				continue
			}
			if hp.data[hx+hy*hp.width] != RC_UNSET_HEIGHT {
				continue
			}
			ai := chf.cells[ax+ay*chf.width].index + GetCon(cs, dir)
			as := &chf.spans[ai]
			hp.data[hx+hy*hp.width] = as.y
			push3(&queue, ax, ay, ai)
		}
	}
}

func seedArrayWithPolyCenter(chf *CompactHeightfield, meshpoly []int, poly int, npoly int, verts []int, bs int, hp *HeightPatch, array *[]int) {
	// Note: Reads to the compact heightfield are offset by border size (bs)
	// since border size offset is already removed from the polymesh vertices.
	offset := []int{0, 0, -1, -1, 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0}
	// Find cell closest to a poly vertex
	startCellX, startCellY, startSpanIndex := 0, 0, -1
	dmin := RC_UNSET_HEIGHT
	for j := 0; j < npoly && dmin > 0; j++ {
		for k := 0; k < 9 && dmin > 0; k++ {
			ax := verts[meshpoly[poly+j]*3+0] + offset[k*2+0]
			ay := verts[meshpoly[poly+j]*3+1]
			az := verts[meshpoly[poly+j]*3+2] + offset[k*2+1]
			if ax < hp.xmin || ax >= hp.xmin+hp.width || az < hp.ymin || az >= hp.ymin+hp.height {
				continue
			}
			c := &chf.cells[(ax+bs)+(az+bs)*chf.width]
			for i, ni := c.index, c.index+c.count; i < ni && dmin > 0; i++ {
				s := &chf.spans[i]
				d := int(math.Abs(float64(ay - s.y)))
				if d < dmin {
					startCellX = ax
					startCellY = az
					startSpanIndex = i
					dmin = d
				}
			}
		}
	}
	// Find center of the polygon
	pcx, pcy := 0, 0
	for j := 0; j < npoly; j++ {
		pcx += verts[meshpoly[poly+j]*3+0]
		pcy += verts[meshpoly[poly+j]*3+2]
	}
	pcx /= npoly
	pcy /= npoly
	*array = []int{}
	*array = append(*array, startCellX, startCellY, startSpanIndex)
	dirs := []int{0, 1, 2, 3}
	for i := 0; i < hp.width*hp.height; i++ {
		hp.data[i] = 0
	}
	// DFS to move to the center. Note that we need a DFS here and can not just move
	// directly towards the center without recording intermediate nodes, even though the polygons
	// are convex. In very rare we can get stuck due to contour simplification if we do not
	// record nodes.
	cx, cy, ci := -1, -1, -1
	for {
		if len(*array) < 3 {
			println("Walk towards polygon center failed to reach center")
			break
		}
		ci = (*array)[len(*array)-1]
		cy = (*array)[len(*array)-2]
		cx = (*array)[len(*array)-3]
		(*array) = (*array)[:len(*array)-3]
		// Check if close to center of the polygon.
		if cx == pcx && cy == pcy {
			break
		}
		// If we are already at the correct X-position, prefer direction
		// directly towards the center in the Y-axis; otherwise prefer
		// direction in the X-axis
		directDir := 0
		if cx == pcx {
			v := -1
			if pcy > cy {
				v = 1
			}
			directDir = rcGetDirForOffset(0, v)
		} else {
			v := -1
			if pcx > cx {
				v = 1
			}
			directDir = rcGetDirForOffset(v, 0)
		}
		// Push the direct dir last so we start with this on next iteration
		dirs[3], dirs[directDir] = dirs[directDir], dirs[3]
		cs := &chf.spans[ci]
		for i := 0; i < 4; i++ {
			dir := dirs[i]
			if GetCon(cs, dir) == RC_NOT_CONNECTED {
				continue
			}
			newX := cx + GetDirOffsetX(dir)
			newY := cy + GetDirOffsetY(dir)
			hpx := newX - hp.xmin
			hpy := newY - hp.ymin
			if hpx < 0 || hpx >= hp.width || hpy < 0 || hpy >= hp.height {
				continue
			}
			if hp.data[hpx+hpy*hp.width] != 0 {
				continue
			}
			hp.data[hpx+hpy*hp.width] = 1
			*array = append(*array, newX, newY, chf.cells[(newX+bs)+(newY+bs)*chf.width].index+GetCon(cs, dir))
		}
		dirs[3], dirs[directDir] = dirs[directDir], dirs[3]
	}
	*array = []int{}
	// getHeightData seeds are given in coordinates with borders
	*array = append(*array, cx+bs, cy+bs, ci)
	for i := 0; i < hp.width*hp.height; i++ {
		hp.data[i] = RC_UNSET_HEIGHT
	}
	cs := &chf.spans[ci]
	hp.data[cx-hp.xmin+(cy-hp.ymin)*hp.width] = cs.y
}

func push3(queue *[]int, v1 int, v2 int, v3 int) {
	*queue = append(*queue, v1, v2, v3)
}
