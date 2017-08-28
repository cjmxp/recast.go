package recast

import (
	"math"
	"strconv"
)

func BuildContours(chf *CompactHeightfield, maxError float32, maxEdgeLen int, buildFlags int) *ContourSet {
	w := chf.width
	h := chf.height
	borderSize := chf.borderSize
	cset := &ContourSet{}
	cset.bmin = make([]float32, 3)
	cset.bmax = make([]float32, 3)
	copy3(cset.bmin, 0, chf.bmin, 0)
	copy3(cset.bmax, 0, chf.bmax, 0)
	if borderSize > 0 {
		// If the heightfield was build with bordersize, remove the offset.
		pad := float32(borderSize) * chf.cs
		cset.bmin[0] += pad
		cset.bmin[2] += pad
		cset.bmax[0] -= pad
		cset.bmax[2] -= pad
	}
	cset.cs = chf.cs
	cset.ch = chf.ch
	cset.width = chf.width - chf.borderSize*2
	cset.height = chf.height - chf.borderSize*2
	cset.borderSize = chf.borderSize
	cset.maxError = maxError

	flags := make([]int, chf.spanCount)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := &chf.cells[x+y*w]
			for i, ni := c.index, c.index+c.count; i < ni; i++ {
				res := 0
				s := &chf.spans[i]
				if chf.spans[i].reg == 0 || (chf.spans[i].reg&RC_BORDER_REG) != 0 {
					flags[i] = 0
					continue
				}
				for dir := 0; dir < 4; dir++ {
					r := 0
					if GetCon(s, dir) != RC_NOT_CONNECTED {
						ax := x + GetDirOffsetX(dir)
						ay := y + GetDirOffsetY(dir)
						ai := chf.cells[ax+ay*w].index + GetCon(s, dir)
						r = chf.spans[ai].reg
					}
					if r == chf.spans[i].reg {
						res |= (1 << uint8(dir))
					}
				}
				flags[i] = res ^ 0xf // Inverse, mark non connected edges.
			}
		}
	}
	verts := []int{}
	simplified := []int{}
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := &chf.cells[x+y*w]
			for i, ni := c.index, c.index+c.count; i < ni; i++ {
				if flags[i] == 0 || flags[i] == 0xf {
					flags[i] = 0
					continue
				}
				reg := chf.spans[i].reg
				if reg == 0 || (reg&RC_BORDER_REG) != 0 {
					continue
				}

				area := chf.areas[i]
				verts = []int{}
				simplified = []int{}
				walkContour6(x, y, i, chf, flags, &verts)
				simplifyContour(verts, &simplified, maxError, maxEdgeLen, buildFlags)
				removeDegenerateSegments(&simplified)
				// Store region->contour remap info.
				// Create contour.
				if len(simplified)/4 >= 3 {
					cont := &Contour{}
					cset.conts = append(cset.conts, cont)
					cont.nverts = len(simplified) / 4
					cont.verts = make([]int, len(simplified))
					copy(cont.verts, simplified)
					if borderSize > 0 {
						// If the heightfield was build with bordersize, remove the offset.
						for j := 0; j < cont.nverts; j++ {
							cont.verts[j*4] -= borderSize
							cont.verts[j*4+2] -= borderSize
						}
					}
					cont.nrverts = len(verts) / 4
					cont.rverts = make([]int, len(verts))
					copy(cont.rverts, verts)
					if borderSize > 0 {
						// If the heightfield was build with bordersize, remove the offset.
						for j := 0; j < cont.nrverts; j++ {
							cont.rverts[j*4] -= borderSize
							cont.rverts[j*4+2] -= borderSize
						}
					}
					cont.reg = reg
					cont.area = area
				}
			}
		}
	}

	// Merge holes if needed.
	if len(cset.conts) > 0 {
		// Calculate winding of all polygons.
		winding := make([]int, len(cset.conts))
		nholes := 0
		for i := 0; i < len(cset.conts); i++ {
			cont := cset.conts[i]
			// If the contour is wound backwards, it is a hole.
			winding[i] = 1
			if calcAreaOfPolygon2D(cont.verts, cont.nverts) < 0 {
				winding[i] = -1
			}
			if winding[i] < 0 {
				nholes++
			}
		}
		if nholes > 0 {
			// Collect outline contour and holes contours per region.
			// We assume that there is one outline and multiple holes.
			nregions := chf.maxRegions + 1
			regions := make([]*ContourRegion, nregions)
			for i := 0; i < nregions; i++ {
				regions[i] = &ContourRegion{}
			}
			for i := 0; i < len(cset.conts); i++ {
				cont := cset.conts[i]
				// Positively would contours are outlines, negative holes.
				if winding[i] > 0 {
					if regions[cont.reg].outline != nil {
						panic("rcBuildContours: Multiple outlines for region " + strconv.Itoa(cont.reg) + ".")
					}
					regions[cont.reg].outline = cont
				} else {
					regions[cont.reg].nholes++
				}
			}
			for i := 0; i < nregions; i++ {
				if regions[i].nholes > 0 {
					regions[i].holes = make([]*ContourHole, regions[i].nholes)
					for nh := 0; nh < regions[i].nholes; nh++ {
						regions[i].holes[nh] = &ContourHole{}
					}
					regions[i].nholes = 0
				}
			}
			for i := 0; i < len(cset.conts); i++ {
				cont := cset.conts[i]
				reg := regions[cont.reg]
				if winding[i] < 0 {
					reg.holes[reg.nholes].contour = cont
					reg.nholes++
				}
			}
			// Finally merge each regions holes into the outline.
			for i := 0; i < nregions; i++ {
				reg := regions[i]
				if reg.nholes == 0 {
					continue
				}
				if reg.outline != nil {
					mergeRegionHoles(reg)
				} else {
					// The region does not have an outline.
					// This can happen if the contour becaomes selfoverlapping because of
					// too aggressive simplification settings.
					panic("rcBuildContours: Bad outline for region " + strconv.Itoa(i) + ", contour simplification is likely too aggressive.")
				}
			}
		}
	}
	return cset
}

func walkContour6(x int, y int, i int, chf *CompactHeightfield, flags []int, points *[]int) {
	// Choose the first non-connected edge
	dir := 0
	for (flags[i] & (1 << uint(dir))) == 0 {
		dir++
	}
	startDir := dir
	starti := i
	area := chf.areas[i]
	iter := 0
	for {
		iter++
		if iter >= 40000 {
			break
		}
		if (flags[i] & (1 << uint(dir))) != 0 {
			// Choose the edge corner
			isBorderVertex := false
			isAreaBorder := false
			px := x
			py := getCornerHeight(x, y, i, dir, chf, isBorderVertex)
			pz := y
			switch dir {
			case 0:
				pz++
				break
			case 1:
				px++
				pz++
				break
			case 2:
				px++
				break
			}
			r := 0
			s := &chf.spans[i]
			if GetCon(s, dir) != RC_NOT_CONNECTED {
				ax := x + GetDirOffsetX(dir)
				ay := y + GetDirOffsetY(dir)
				ai := chf.cells[ax+ay*chf.width].index + GetCon(s, dir)
				r = chf.spans[ai].reg
				if area != chf.areas[ai] {
					isAreaBorder = true
				}
			}
			if isBorderVertex {
				r |= RC_BORDER_VERTEX
			}
			if isAreaBorder {
				r |= RC_AREA_BORDER
			}
			*points = append(*points, px, py, pz, r)
			flags[i] &= ^(1 << uint(dir)) // Remove visited edges
			dir = (dir + 1) & 0x3         // Rotate CW
		} else {
			ni := -1
			nx := x + GetDirOffsetX(dir)
			ny := y + GetDirOffsetY(dir)
			s := &chf.spans[i]
			if GetCon(s, dir) != RC_NOT_CONNECTED {
				nc := &chf.cells[nx+ny*chf.width]
				ni = nc.index + GetCon(s, dir)
			}
			if ni == -1 {
				// Should not happen.
				return
			}
			x = nx
			y = ny
			i = ni
			dir = (dir + 3) & 0x3 // Rotate CCW
		}
		if starti == i && startDir == dir {
			break
		}
	}
}

func getCornerHeight(x int, y int, i int, dir int, chf *CompactHeightfield, isBorderVertex bool) int {
	s := &chf.spans[i]
	ch := s.y
	dirp := (dir + 1) & 0x3
	regs := []int{0, 0, 0, 0}
	// Combine region and area codes in order to prevent
	// border vertices which are in between two areas to be removed.
	regs[0] = chf.spans[i].reg | (chf.areas[i] << 16)
	if GetCon(s, dir) != RC_NOT_CONNECTED {
		ax := x + GetDirOffsetX(dir)
		ay := y + GetDirOffsetY(dir)
		ai := chf.cells[ax+ay*chf.width].index + GetCon(s, dir)
		as := &chf.spans[ai]
		ch = int(math.Max(float64(ch), float64(as.y)))
		regs[1] = chf.spans[ai].reg | (chf.areas[ai] << 16)
		if GetCon(as, dirp) != RC_NOT_CONNECTED {
			ax2 := ax + GetDirOffsetX(dirp)
			ay2 := ay + GetDirOffsetY(dirp)
			ai2 := chf.cells[ax2+ay2*chf.width].index + GetCon(as, dirp)
			as2 := &chf.spans[ai2]
			ch = int(math.Max(float64(ch), float64(as2.y)))
			regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16)
		}
	}
	if GetCon(s, dirp) != RC_NOT_CONNECTED {
		ax := x + GetDirOffsetX(dirp)
		ay := y + GetDirOffsetY(dirp)
		ai := chf.cells[ax+ay*chf.width].index + GetCon(s, dirp)
		as := &chf.spans[ai]
		ch = int(math.Max(float64(ch), float64(as.y)))
		regs[3] = chf.spans[ai].reg | (chf.areas[ai] << 16)
		if GetCon(as, dir) != RC_NOT_CONNECTED {
			ax2 := ax + GetDirOffsetX(dir)
			ay2 := ay + GetDirOffsetY(dir)
			ai2 := chf.cells[ax2+ay2*chf.width].index + GetCon(as, dir)
			as2 := &chf.spans[ai2]
			ch = int(math.Max(float64(ch), float64(as2.y)))
			regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16)
		}
	}
	// Check if the vertex is special edge vertex, these vertices will be removed later.
	for j := 0; j < 4; j++ {
		a := j
		b := (j + 1) & 0x3
		c := (j + 2) & 0x3
		d := (j + 3) & 0x3

		// The vertex is a border vertex there are two same exterior cells in a row,
		// followed by two interior cells and none of the regions are out of bounds.
		twoSameExts := (regs[a]&regs[b]&RC_BORDER_REG) != 0 && regs[a] == regs[b]
		twoInts := ((regs[c] | regs[d]) & RC_BORDER_REG) == 0
		intsSameArea := (regs[c] >> 16) == (regs[d] >> 16)
		noZeros := regs[a] != 0 && regs[b] != 0 && regs[c] != 0 && regs[d] != 0
		if twoSameExts && twoInts && intsSameArea && noZeros {
			isBorderVertex = true
			break
		}
	}
	return ch
}

func simplifyContour(points []int, simplified *[]int, maxError float32, maxEdgeLen int, buildFlags int) {
	// Add initial points.
	hasConnections := false
	for i := 0; i < len(points); i += 4 {
		if (points[i+3] & RC_CONTOUR_REG_MASK) != 0 {
			hasConnections = true
			break
		}
	}
	if hasConnections {
		// The contour has some portals to other regions.
		// Add a new point to every location where the region changes.
		for i, ni := 0, len(points)/4; i < ni; i++ {
			ii := (i + 1) % ni
			differentRegs := (points[i*4+3] & RC_CONTOUR_REG_MASK) != (points[ii*4+3] & RC_CONTOUR_REG_MASK)
			areaBorders := (points[i*4+3] & RC_AREA_BORDER) != (points[ii*4+3] & RC_AREA_BORDER)
			if differentRegs || areaBorders {
				*simplified = append(*simplified, points[i*4+0], points[i*4+1], points[i*4+2], i)
			}
		}
	}
	if len(*simplified) == 0 {
		// If there is no connections at all,
		// create some initial points for the simplification process.
		// Find lower-left and upper-right vertices of the contour.
		llx := points[0]
		lly := points[1]
		llz := points[2]
		lli := 0
		urx := points[0]
		ury := points[1]
		urz := points[2]
		uri := 0
		for i := 0; i < len(points); i += 4 {
			x := points[i+0]
			y := points[i+1]
			z := points[i+2]
			if x < llx || (x == llx && z < llz) {
				llx = x
				lly = y
				llz = z
				lli = i / 4
			}
			if x > urx || (x == urx && z > urz) {
				urx = x
				ury = y
				urz = z
				uri = i / 4
			}
		}
		*simplified = append(*simplified, llx, lly, llz, lli, urx, ury, urz, uri)
	}
	// Add points until all raw points are within
	// error tolerance to the simplified shape.
	pn := len(points) / 4
	for i := 0; i < len(*simplified)/4; {
		ii := (i + 1) % (len(*simplified) / 4)

		ax := (*simplified)[i*4+0]
		az := (*simplified)[i*4+2]
		ai := (*simplified)[i*4+3]

		bx := (*simplified)[ii*4+0]
		bz := (*simplified)[ii*4+2]
		bi := (*simplified)[ii*4+3]

		// Find maximum deviation from the segment.
		maxd := float32(0)
		maxi := -1
		ci, cinc, endi := 0, 0, 0
		// Traverse the segment in lexilogical order so that the
		// max deviation is calculated similarly when traversing
		// opposite segments.
		if bx > ax || (bx == ax && bz > az) {
			cinc = 1
			ci = (ai + cinc) % pn
			endi = bi
		} else {
			cinc = pn - 1
			ci = (bi + cinc) % pn
			endi = ai
			ax, bx = bx, ax
			az, bz = bz, az

		}
		// Tessellate only outer edges or edges between areas.
		if (points[ci*4+3]&RC_CONTOUR_REG_MASK) == 0 || (points[ci*4+3]&RC_AREA_BORDER) != 0 {
			for ci != endi {
				d := distancePtSeg(points[ci*4+0], points[ci*4+2], ax, az, bx, bz)
				if d > maxd {
					maxd = d
					maxi = ci
				}
				ci = (ci + cinc) % pn
			}
		}
		// If the max deviation is larger than accepted error,
		// add new point, else continue to next segment.
		if maxi != -1 && maxd > (maxError*maxError) {
			// Add the point.
			v := []int{points[maxi*4+0], points[maxi*4+1], points[maxi*4+2], maxi}
			e := append([]int{}, (*simplified)[(i+1)*4+0:]...)
			*simplified = append((*simplified)[:(i+1)*4+0], v...)
			*simplified = append(*simplified, e...)
		} else {
			i++
		}
	}
	// Split too long edges.
	if maxEdgeLen > 0 && (buildFlags&(RC_CONTOUR_TESS_WALL_EDGES|RC_CONTOUR_TESS_AREA_EDGES)) != 0 {
		for i := 0; i < len(*simplified)/4; {
			ii := (i + 1) % (len(*simplified) / 4)

			ax := (*simplified)[i*4+0]
			az := (*simplified)[i*4+2]
			ai := (*simplified)[i*4+3]

			bx := (*simplified)[ii*4+0]
			bz := (*simplified)[ii*4+2]
			bi := (*simplified)[ii*4+3]

			// Find maximum deviation from the segment.
			maxi := -1
			ci := (ai + 1) % pn
			// Tessellate only outer edges or edges between areas.
			tess := false
			// Wall edges.
			if (buildFlags&RC_CONTOUR_TESS_WALL_EDGES) != 0 && (points[ci*4+3]&RC_CONTOUR_REG_MASK) == 0 {
				tess = true
			}
			// Edges between areas.
			if (buildFlags&RC_CONTOUR_TESS_AREA_EDGES) != 0 && (points[ci*4+3]&RC_AREA_BORDER) != 0 {
				tess = true
			}
			if tess {
				dx := bx - ax
				dz := bz - az
				if dx*dx+dz*dz > maxEdgeLen*maxEdgeLen {
					// Round based on the segments in lexilogical order so that the
					// max tesselation is consistent regardles in which direction
					// segments are traversed.
					n := bi - ai
					if bi < ai {
						n = bi + pn - ai
					}
					if n > 1 {
						if bx > ax || (bx == ax && bz > az) {
							maxi = (ai + n/2) % pn
						} else {
							maxi = (ai + (n+1)/2) % pn
						}
					}
				}
			}
			// If the max deviation is larger than accepted error,
			// add new point, else continue to next segment.
			if maxi != -1 {
				// Add the point.
				v := []int{points[maxi*4+0], points[maxi*4+1], points[maxi*4+2], maxi}
				e := append([]int{}, (*simplified)[(i+1)*4+0:]...)
				*simplified = append((*simplified)[:(i+1)*4+0], v...)
				*simplified = append(*simplified, e...)
			} else {
				i++
			}
		}
	}
	for i := 0; i < len(*simplified)/4; i++ {
		// The edge vertex flag is take from the current raw point,
		// and the neighbour region is take from the next raw point.
		ai := ((*simplified)[i*4+3] + 1) % pn
		bi := (*simplified)[i*4+3]
		(*simplified)[i*4+3] = (points[ai*4+3] & (RC_CONTOUR_REG_MASK | RC_AREA_BORDER)) | (points[bi*4+3] & RC_BORDER_VERTEX)
	}
}
func distancePtSeg(x, z, px, pz, qx, qz int) float32 {
	pqx := float32(qx - px)
	pqz := float32(qz - pz)
	dx := float32(x - px)
	dz := float32(z - pz)
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
	dx = float32(px) + t*pqx - float32(x)
	dz = float32(pz) + t*pqz - float32(z)
	return dx*dx + dz*dz
}
func removeDegenerateSegments(simplified *[]int) {
	// Remove adjacent vertices which are equal on xz-plane,
	// or else the triangulator will get confused.
	npts := len(*simplified) / 4
	for i := 0; i < npts; i++ {
		ni := next(i, npts)
		//if (vequal(&simplified[i*4], &simplified[ni*4]))
		if (*simplified)[i*4] == (*simplified)[ni*4] && (*simplified)[i*4+2] == (*simplified)[ni*4+2] {
			// Degenerate segment, remove.
			*simplified = append((*simplified)[:i*4], (*simplified)[i*4+4:]...)
			npts--
		}
	}
}
func calcAreaOfPolygon2D(verts []int, nverts int) int {
	area := 0
	for i, j := 0, nverts-1; i < nverts; j, i = i, i+1 {
		vi := i * 4
		vj := j * 4
		area += verts[vi+0]*verts[vj+2] - verts[vj+0]*verts[vi+2]
	}
	return (area + 1) / 2
}
func mergeRegionHoles(region *ContourRegion) {
	// Sort holes from left to right.
	for i := 0; i < region.nholes; i++ {
		minleft := findLeftMostVertex(region.holes[i].contour)
		region.holes[i].minx = minleft[0]
		region.holes[i].minz = minleft[1]
		region.holes[i].leftmost = minleft[2]
	}
	isort := &ISort{}
	isort.Data = region.holes
	isort.Call = func(a interface{}, b interface{}) bool {
		_a := a.(*ContourHole)
		_b := b.(*ContourHole)
		if _a.minx == _b.minx {
			if _a.minz < _b.minz {
				return true
			}
		} else {
			if _a.minx < _b.minx {
				return true
			}
		}
		return false
	}
	isort.Sort()
	isort.Free()
	maxVerts := region.outline.nverts
	for i := 0; i < region.nholes; i++ {
		maxVerts += region.holes[i].contour.nverts
	}
	diags := make([]*PotentialDiagonal, maxVerts)
	for pd := 0; pd < maxVerts; pd++ {
		diags[pd] = &PotentialDiagonal{}
	}
	outline := region.outline
	// Merge holes into the outline one by one.
	for i := 0; i < region.nholes; i++ {
		hole := region.holes[i].contour
		index := -1
		bestVertex := region.holes[i].leftmost
		for iter := 0; iter < hole.nverts; iter++ {
			// Find potential diagonals.
			// The 'best' vertex must be in the cone described by 3 cosequtive vertices of the outline.
			// ..o j-1
			//   |
			//   |   * best
			//   |
			// j o-----o j+1
			//         :
			ndiags := 0
			corner := bestVertex * 4
			for j := 0; j < outline.nverts; j++ {
				if inCone(j, outline.nverts, outline.verts, corner, hole.verts) {
					dx := outline.verts[j*4+0] - hole.verts[corner+0]
					dz := outline.verts[j*4+2] - hole.verts[corner+2]
					diags[ndiags].vert = j
					diags[ndiags].dist = dx*dx + dz*dz
					ndiags++
				}
			}
			// Sort potential diagonals by distance, we want to make the connection as short as possible.
			isort.Data = diags[:ndiags]
			isort.Call = func(a interface{}, b interface{}) bool {
				_a := a.(*PotentialDiagonal)
				_b := b.(*PotentialDiagonal)
				if _a.dist < _b.dist {
					return true
				}
				return false
			}
			isort.Stable()
			isort.Free()
			// Find a diagonal that is not intersecting the outline not the remaining holes.
			index = -1
			for j := 0; j < ndiags; j++ {
				pt := diags[j].vert * 4
				intersect := intersectSegCountour(pt, corner, diags[i].vert, outline.nverts, outline.verts, outline.verts, hole.verts)
				for k := i; k < region.nholes && !intersect; k++ {
					intersect = intersectSegCountour(pt, corner, -1, region.holes[k].contour.nverts, region.holes[k].contour.verts, outline.verts, hole.verts)
				}
				if !intersect {
					index = diags[j].vert
					break
				}
			}
			// If found non-intersecting diagonal, stop looking.
			if index != -1 {
				break
			}
			// All the potential diagonals for the current vertex were intersecting, try next vertex.
			bestVertex = (bestVertex + 1) % hole.nverts
		}
		if index == -1 {
			println("mergeHoles: Failed to find merge points for")
			continue
		}
		mergeContours(region.outline, hole, index, bestVertex)
	}
}
func findLeftMostVertex(contour *Contour) []int {
	minx := contour.verts[0]
	minz := contour.verts[2]
	leftmost := 0
	for i := 1; i < contour.nverts; i++ {
		x := contour.verts[i*4+0]
		z := contour.verts[i*4+2]
		if x < minx || (x == minx && z < minz) {
			minx = x
			minz = z
			leftmost = i
		}
	}
	return []int{minx, minz, leftmost}
}
func intersectSegCountour(d0 int, d1 int, i int, n int, verts []int, d0verts []int, d1verts []int) bool {
	// For each edge (k,k+1) of P
	pverts := make([]int, 4*4)
	for g := 0; g < 4; g++ {
		pverts[g] = d0verts[d0+g]
		pverts[4+g] = d1verts[d1+g]
	}
	d0 = 0
	d1 = 4
	for k := 0; k < n; k++ {
		k1 := next(k, n)
		// Skip edges incident to i.
		if i == k || i == k1 {
			continue
		}

		p0 := k * 4
		p1 := k1 * 4
		for g := 0; g < 4; g++ {
			pverts[8+g] = verts[p0+g]
			pverts[12+g] = verts[p1+g]
		}
		p0 = 8
		p1 = 12
		if vequal(pverts, d0, p0) || vequal(pverts, d1, p0) || vequal(pverts, d0, p1) || vequal(pverts, d1, p1) {
			continue
		}
		if intersect(pverts, d0, d1, p0, p1) {
			return true
		}
	}
	return false
}
func inCone(i int, n int, verts []int, pj int, vertpj []int) bool {
	pi := i * 4
	pi1 := next(i, n) * 4
	pin1 := prev(i, n) * 4
	pverts := make([]int, 4*4)
	for g := 0; g < 4; g++ {
		pverts[g] = verts[pi+g]
		pverts[4+g] = verts[pi1+g]
		pverts[8+g] = verts[pin1+g]
		pverts[12+g] = vertpj[pj+g]
	}
	pi = 0
	pi1 = 4
	pin1 = 8
	pj = 12
	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	if leftOn(pverts, pin1, pi, pi1) {
		return left(pverts, pi, pj, pin1) && left(pverts, pj, pi, pi1)
	}

	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	return !(leftOn(pverts, pi, pj, pi1) && leftOn(pverts, pj, pi, pin1))
}
func mergeContours(ca *Contour, cb *Contour, ia int, ib int) {
	maxVerts := ca.nverts + cb.nverts + 2
	verts := make([]int, maxVerts*4)
	nv := 0
	// Copy contour A.
	for i := 0; i <= ca.nverts; i++ {
		dst := nv * 4
		src := ((ia + i) % ca.nverts) * 4
		verts[dst+0] = ca.verts[src+0]
		verts[dst+1] = ca.verts[src+1]
		verts[dst+2] = ca.verts[src+2]
		verts[dst+3] = ca.verts[src+3]
		nv++
	}
	// Copy contour B
	for i := 0; i <= cb.nverts; i++ {
		dst := nv * 4
		src := ((ib + i) % cb.nverts) * 4
		verts[dst+0] = cb.verts[src+0]
		verts[dst+1] = cb.verts[src+1]
		verts[dst+2] = cb.verts[src+2]
		verts[dst+3] = cb.verts[src+3]
		nv++
	}
	ca.verts = verts
	ca.nverts = nv

	cb.verts = nil
	cb.nverts = 0
}
