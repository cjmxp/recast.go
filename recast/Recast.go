package recast

import "math"

func calcGridSize(bmin []float32, bmax []float32, cs float32) (w, h int) {
	return int((bmax[0]-bmin[0])/cs + 0.5), int((bmax[2]-bmin[2])/cs + 0.5)
}

func calcTileCount(bmin []float32, bmax []float32, cs float32, tileSize int) (w, h int) {
	gw, gh := calcGridSize(bmin, bmax, cs)
	return int((gw + tileSize - 1) / tileSize), int((gh + tileSize - 1) / tileSize)
}
func MarkWalkableTriangles(walkableSlopeAngle float32, verts []float32, tris []int, nt int) []int {
	areas := make([]int, nt)
	walkableThr := float32(math.Cos(float64(walkableSlopeAngle) / 180.0 * math.Pi))
	norm := [3]float32{}
	for i := 0; i < nt; i++ {
		tri := i * 3
		calcTriNormal(verts, tris[tri], tris[tri+1], tris[tri+2], norm[:])
		// Check if the face is walkable.
		if norm[1] > walkableThr {
			areas[i] = RC_WALKABLE_AREA
		}
	}
	return areas
}
func calcTriNormal(verts []float32, v0 int, v1 int, v2 int, norm []float32) {
	e0 := [3]float32{}
	e1 := [3]float32{}
	sub(e0[:], verts, v1*3, v0*3)
	sub(e1[:], verts, v2*3, v0*3)
	cross(norm, e0[:], e1[:])
	normalize(norm)
}
func BuildCompactHeightfield(walkableHeight int, walkableClimb int, hf *Heightfield) *CompactHeightfield {
	chf := &CompactHeightfield{}
	w := hf.width
	h := hf.height
	spanCount := getHeightFieldSpanCount(hf)
	// Fill in header.
	chf.width = w
	chf.height = h
	chf.spanCount = spanCount
	chf.walkableHeight = walkableHeight
	chf.walkableClimb = walkableClimb
	chf.maxRegions = 0
	chf.bmin = make([]float32, 3)
	chf.bmax = make([]float32, 3)
	copy3(chf.bmin, 0, hf.bmin, 0)
	copy3(chf.bmax, 0, hf.bmax, 0)
	chf.bmax[1] += float32(walkableHeight) * hf.ch
	chf.cs = hf.cs
	chf.ch = hf.ch
	chf.cells = make([]CompactCell, w*h)
	chf.spans = make([]CompactSpan, spanCount)
	chf.areas = make([]int, spanCount)
	// Fill in cells and spans.
	idx := 0
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			s := hf.spans[x+y*w]
			// If there are no spans at this cell, just leave the data to index=0, count=0.
			if s == nil {
				continue
			}
			c := &chf.cells[x+y*w]
			c.index = idx
			c.count = 0
			for s != nil {
				if s.area != RC_NULL_AREA {
					bot := s.smax
					top := MAX_HEIGHT
					if s.next != nil {
						top = s.next.smin
					}
					chf.spans[idx].y = clamp_i(bot, 0, 0xffff)
					chf.spans[idx].h = clamp_i(top-bot, 0, 0xff)
					chf.areas[idx] = s.area
					idx++
					c.count++
				}
				s = s.next
			}
		}
	}
	// Find neighbour connections.
	tooHighNeighbour := 0
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := &chf.cells[x+y*w]
			for i, ni := c.index, c.index+c.count; i < ni; i++ {
				s := &chf.spans[i]
				for dir := 0; dir < 4; dir++ {
					SetCon(s, dir, RC_NOT_CONNECTED)
					nx := x + GetDirOffsetX(dir)
					ny := y + GetDirOffsetY(dir)
					// First check that the neighbour cell is in bounds.
					if nx < 0 || ny < 0 || nx >= w || ny >= h {
						continue
					}
					// Iterate over all neighbour spans and check if any of the is
					// accessible from current cell.
					nc := &chf.cells[nx+ny*w]
					for k, nk := nc.index, nc.index+nc.count; k < nk; k++ {
						ns := &chf.spans[k]
						bot := int(math.Max(float64(s.y), float64(ns.y)))
						top := int(math.Min(float64(s.y+s.h), float64(ns.y+ns.h)))
						// Check that the gap between the spans is walkable,
						// and that the climb height between the gaps is not too high.
						if (top-bot) >= walkableHeight && int(math.Abs(float64(ns.y-s.y))) <= walkableClimb {
							// Mark direction as walkable.
							lidx := k - nc.index
							if lidx < 0 || lidx > MAX_LAYERS {
								tooHighNeighbour = int(math.Max(float64(tooHighNeighbour), float64(lidx)))
								continue
							}
							SetCon(s, dir, lidx)
							break
						}
					}
				}
			}
		}
	}
	return chf
}
func getHeightFieldSpanCount(hf *Heightfield) int {
	w := hf.width
	h := hf.height
	spanCount := 0
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			for s := hf.spans[x+y*w]; s != nil; s = s.next {
				if s.area != RC_NULL_AREA {
					spanCount++
				}
			}
		}
	}
	return spanCount
}
