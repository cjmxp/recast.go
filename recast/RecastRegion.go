package recast

import (
	"math"
	"reflect"
	"unsafe"
)

func BuildDistanceField(chf *CompactHeightfield) {
	src := make([]int, chf.spanCount)
	chf.maxDistance = calculateDistanceField(chf, src)
	// Blur
	src = boxBlur(chf, 1, src)
	// Store distance.
	chf.dist = src
}

func calculateDistanceField(chf *CompactHeightfield, src []int) int {
	w := chf.width
	h := chf.height
	// Init distance and points.
	for i := 0; i < chf.spanCount; i++ {
		src[i] = 0xffff
	}
	// Mark boundary cells.
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := &chf.cells[x+y*w]
			for i, ni := c.index, c.index+c.count; i < ni; i++ {
				s := &chf.spans[i]
				area := chf.areas[i]
				nc := 0
				for dir := 0; dir < 4; dir++ {
					if GetCon(s, dir) != RC_NOT_CONNECTED {
						ax := x + GetDirOffsetX(dir)
						ay := y + GetDirOffsetY(dir)
						ai := chf.cells[ax+ay*w].index + GetCon(s, dir)
						if area == chf.areas[ai] {
							nc++
						}
					}
				}
				if nc != 4 {
					src[i] = 0
				}
			}
		}
	}
	// Pass 1
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := &chf.cells[x+y*w]
			for i, ni := c.index, c.index+c.count; i < ni; i++ {
				s := &chf.spans[i]
				if GetCon(s, 0) != RC_NOT_CONNECTED {
					// (-1,0)
					ax := x + GetDirOffsetX(0)
					ay := y + GetDirOffsetY(0)
					ai := chf.cells[ax+ay*w].index + GetCon(s, 0)
					as := &chf.spans[ai]
					if src[ai]+2 < src[i] {
						src[i] = src[ai] + 2
					}
					// (-1,-1)
					if GetCon(as, 3) != RC_NOT_CONNECTED {
						aax := ax + GetDirOffsetX(3)
						aay := ay + GetDirOffsetY(3)
						aai := chf.cells[aax+aay*w].index + GetCon(as, 3)
						if src[aai]+3 < src[i] {
							src[i] = src[aai] + 3
						}
					}
				}
				if GetCon(s, 3) != RC_NOT_CONNECTED {
					// (0,-1)
					ax := x + GetDirOffsetX(3)
					ay := y + GetDirOffsetY(3)
					ai := chf.cells[ax+ay*w].index + GetCon(s, 3)
					as := &chf.spans[ai]
					if src[ai]+2 < src[i] {
						src[i] = src[ai] + 2
					}
					// (1,-1)
					if GetCon(as, 2) != RC_NOT_CONNECTED {
						aax := ax + GetDirOffsetX(2)
						aay := ay + GetDirOffsetY(2)
						aai := chf.cells[aax+aay*w].index + GetCon(as, 2)
						if src[aai]+3 < src[i] {
							src[i] = src[aai] + 3
						}
					}
				}
			}
		}
	}
	// Pass 2
	for y := h - 1; y >= 0; y-- {
		for x := w - 1; x >= 0; x-- {
			c := &chf.cells[x+y*w]
			for i, ni := c.index, c.index+c.count; i < ni; i++ {
				s := &chf.spans[i]
				if GetCon(s, 2) != RC_NOT_CONNECTED {
					// (1,0)
					ax := x + GetDirOffsetX(2)
					ay := y + GetDirOffsetY(2)
					ai := chf.cells[ax+ay*w].index + GetCon(s, 2)
					as := &chf.spans[ai]
					if src[ai]+2 < src[i] {
						src[i] = src[ai] + 2
					}
					// (1,1)
					if GetCon(as, 1) != RC_NOT_CONNECTED {
						aax := ax + GetDirOffsetX(1)
						aay := ay + GetDirOffsetY(1)
						aai := chf.cells[aax+aay*w].index + GetCon(as, 1)
						if src[aai]+3 < src[i] {
							src[i] = src[aai] + 3
						}
					}
				}
				if GetCon(s, 1) != RC_NOT_CONNECTED {
					// (0,1)
					ax := x + GetDirOffsetX(1)
					ay := y + GetDirOffsetY(1)
					ai := chf.cells[ax+ay*w].index + GetCon(s, 1)
					as := &chf.spans[ai]
					if src[ai]+2 < src[i] {
						src[i] = src[ai] + 2
					}
					// (-1,1)
					if GetCon(as, 0) != RC_NOT_CONNECTED {
						aax := ax + GetDirOffsetX(0)
						aay := ay + GetDirOffsetY(0)
						aai := chf.cells[aax+aay*w].index + GetCon(as, 0)
						if src[aai]+3 < src[i] {
							src[i] = src[aai] + 3
						}
					}
				}
			}
		}
	}
	maxDist := 0
	for i := 0; i < chf.spanCount; i++ {
		maxDist = int(math.Max(float64(src[i]), float64(maxDist)))
	}
	return maxDist
}

func boxBlur(chf *CompactHeightfield, thr int, src []int) []int {
	w := chf.width
	h := chf.height
	dst := make([]int, chf.spanCount)
	thr *= 2
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := &chf.cells[x+y*w]
			for i, ni := c.index, c.index+c.count; i < ni; i++ {
				s := &chf.spans[i]
				cd := src[i]
				if cd <= thr {
					dst[i] = cd
					continue
				}
				d := cd
				for dir := 0; dir < 4; dir++ {
					if GetCon(s, dir) != RC_NOT_CONNECTED {
						ax := x + GetDirOffsetX(dir)
						ay := y + GetDirOffsetY(dir)
						ai := chf.cells[ax+ay*w].index + GetCon(s, dir)
						d += src[ai]
						as := &chf.spans[ai]
						dir2 := (dir + 1) & 0x3
						if GetCon(as, dir2) != RC_NOT_CONNECTED {
							ax2 := ax + GetDirOffsetX(dir2)
							ay2 := ay + GetDirOffsetY(dir2)
							ai2 := chf.cells[ax2+ay2*w].index + GetCon(as, dir2)
							d += src[ai2]
						} else {
							d += cd
						}
					} else {
						d += cd * 2
					}
				}
				dst[i] = ((d + 5) / 9)
			}
		}
	}
	return dst
}
func BuildRegions(chf *CompactHeightfield, borderSize int, minRegionArea int, mergeRegionArea int) {
	w := chf.width
	h := chf.height
	lvlStacks := make([][]int, 8)
	for i := 0; i < NB_STACKS; i++ {
		lvlStacks[i] = []int{}
	}
	stack := []int{}
	srcReg := make([]int, chf.spanCount)
	srcDist := make([]int, chf.spanCount)
	dstReg := make([]int, chf.spanCount)
	dstDist := make([]int, chf.spanCount)
	regionId := 1
	level := (chf.maxDistance + 1) & ^1
	// TODO: Figure better formula, expandIters defines how much the
	// watershed "overflows" and simplifies the regions. Tying it to
	// agent radius was usually good indication how greedy it could be.
	//		const int expandIters = 4 + walkableRadius * 2;
	expandIters := 8
	if borderSize > 0 {
		// Make sure border will not overflow.
		bw := int(math.Min(float64(w), float64(borderSize)))
		bh := int(math.Min(float64(h), float64(borderSize)))
		// Paint regions
		paintRectRegion(0, bw, 0, h, regionId|RC_BORDER_REG, chf, srcReg)
		regionId++
		paintRectRegion(w-bw, w, 0, h, regionId|RC_BORDER_REG, chf, srcReg)
		regionId++
		paintRectRegion(0, w, 0, bh, regionId|RC_BORDER_REG, chf, srcReg)
		regionId++
		paintRectRegion(0, w, h-bh, h, regionId|RC_BORDER_REG, chf, srcReg)
		regionId++
		chf.borderSize = borderSize
	}
	sId := -1
	for level > 0 {
		if level >= 2 {
			level -= 2
		} else {
			level = 0
		}
		sId = (sId + 1) & (NB_STACKS - 1)
		if sId == 0 {
			sortCellsByLevel(level, chf, srcReg, NB_STACKS, &lvlStacks, 1)
		} else {
			appendStacks(lvlStacks[sId-1], &lvlStacks[sId], srcReg) // copy left overs from last level
		}
		// Expand current regions until no empty connected cells found.
		re := expandRegions(expandIters, level, chf, srcReg, srcDist, dstReg, dstDist, &lvlStacks[sId], false)
		if (*reflect.SliceHeader)(unsafe.Pointer(&re)).Data != (*reflect.SliceHeader)(unsafe.Pointer(&srcReg)).Data {
			srcReg, dstReg = dstReg, srcReg
			srcDist, dstDist = dstDist, srcDist
		}
		// Mark new regions with IDs.
		for j := 0; j < len(lvlStacks[sId]); j += 3 {
			x := lvlStacks[sId][j]
			y := lvlStacks[sId][j+1]
			i := lvlStacks[sId][j+2]
			if i >= 0 && srcReg[i] == 0 {
				if floodRegion(x, y, i, level, regionId, chf, srcReg, srcDist, &stack) {
					regionId++
				}
			}
		}
	}
	// Expand current regions until no empty connected cells found.
	re := expandRegions(expandIters*8, 0, chf, srcReg, srcDist, dstReg, dstDist, &stack, true)
	if (*reflect.SliceHeader)(unsafe.Pointer(&re)).Data != (*reflect.SliceHeader)(unsafe.Pointer(&srcReg)).Data {
		srcReg, dstReg = dstReg, srcReg
		srcDist, dstDist = dstDist, srcDist
	}
	// Merge regions and filter out smalle regions.
	overlaps := []int{}
	chf.maxRegions = mergeAndFilterRegions(minRegionArea, mergeRegionArea, regionId, chf, srcReg, overlaps)
	// Monotone partitioning does not generate overlapping regions.
	for i := 0; i < chf.spanCount; i++ {
		chf.spans[i].reg = srcReg[i]
	}
}

func paintRectRegion(minx int, maxx int, miny int, maxy int, regId int, chf *CompactHeightfield, srcReg []int) {
	w := chf.width
	for y := miny; y < maxy; y++ {
		for x := minx; x < maxx; x++ {
			c := &chf.cells[x+y*w]
			for i, ni := c.index, c.index+c.count; i < ni; i++ {
				if chf.areas[i] != RC_NULL_AREA {
					srcReg[i] = regId
				}
			}
		}
	}
}

func sortCellsByLevel(startLevel int, chf *CompactHeightfield, srcReg []int, nbStacks int, stacks *[][]int, loglevelsPerStack int) { // the levels per stack (2 in our case) as a bit shift
	w := chf.width
	h := chf.height
	startLevel = startLevel >> uint(loglevelsPerStack)
	for j := 0; j < nbStacks; j++ {
		(*stacks)[j] = []int{}
	}
	// put all cells in the level range into the appropriate stacks
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := &chf.cells[x+y*w]
			for i, ni := c.index, c.index+c.count; i < ni; i++ {
				if chf.areas[i] == RC_NULL_AREA || srcReg[i] != 0 {
					continue
				}
				level := chf.dist[i] >> uint(loglevelsPerStack)
				sId := startLevel - level
				if sId >= nbStacks {
					continue
				}
				if sId < 0 {
					sId = 0
				}
				(*stacks)[sId] = append((*stacks)[sId], x, y, i)
			}
		}
	}
}
func appendStacks(srcStack []int, dstStack *[]int, srcReg []int) {
	for j := 0; j < len(srcStack); j += 3 {
		i := srcStack[j+2]
		if (i < 0) || (srcReg[i] != 0) {
			continue
		}
		*dstStack = append(*dstStack, srcStack[j], srcStack[j+1], srcStack[j+2])
	}

}
func expandRegions(maxIter int, level int, chf *CompactHeightfield, srcReg []int, srcDist []int, dstReg []int, dstDist []int, stack *[]int, fillStack bool) []int {
	w := chf.width
	h := chf.height
	if fillStack {
		// Find cells revealed by the raised level.
		*stack = []int{}
		for y := 0; y < h; y++ {
			for x := 0; x < w; x++ {
				c := &chf.cells[x+y*w]
				for i, ni := c.index, c.index+c.count; i < ni; i++ {
					if chf.dist[i] >= level && srcReg[i] == 0 && chf.areas[i] != RC_NULL_AREA {
						*stack = append(*stack, x, y, i)
					}
				}
			}
		}
	} else { // use cells in the input stack
		// mark all cells which already have a region
		for j := 0; j < len(*stack); j += 3 {
			i := (*stack)[j+2]
			if srcReg[i] != 0 {
				(*stack)[j+2] = -1
			}
		}
	}
	iter := 0
	for len(*stack) > 0 {
		failed := 0
		copy(dstReg[:chf.spanCount], srcReg[:chf.spanCount])
		copy(dstDist[:chf.spanCount], srcDist[:chf.spanCount])
		for j := 0; j < len(*stack); j += 3 {
			x := (*stack)[j+0]
			y := (*stack)[j+1]
			i := (*stack)[j+2]
			if i < 0 {
				failed++
				continue
			}
			r := srcReg[i]
			d2 := 0xffff
			area := chf.areas[i]
			s := &chf.spans[i]
			for dir := 0; dir < 4; dir++ {
				if GetCon(s, dir) == RC_NOT_CONNECTED {
					continue
				}
				ax := x + GetDirOffsetX(dir)
				ay := y + GetDirOffsetY(dir)
				ai := chf.cells[ax+ay*w].index + GetCon(s, dir)
				if chf.areas[ai] != area {
					continue
				}
				if srcReg[ai] > 0 && (srcReg[ai]&RC_BORDER_REG) == 0 {
					if srcDist[ai]+2 < d2 {
						r = srcReg[ai]
						d2 = srcDist[ai] + 2
					}
				}
			}
			if r != 0 {
				(*stack)[j+2] = -1 // mark as used
				dstReg[i] = r
				dstDist[i] = d2
			} else {
				failed++
			}
		}
		// rcSwap source and dest.
		srcReg, dstReg = dstReg, srcReg
		srcDist, dstDist = dstDist, srcDist
		if failed*3 == len(*stack) {
			break
		}
		if level > 0 {
			iter++
			if iter >= maxIter {
				break
			}
		}
	}
	return srcReg
}
func floodRegion(x int, y int, i int, level int, r int, chf *CompactHeightfield, srcReg []int, srcDist []int, stack *[]int) bool {
	w := chf.width
	area := chf.areas[i]
	// Flood fill mark region.
	*stack = []int{}
	*stack = append(*stack, x, y, i)
	srcReg[i] = r
	srcDist[i] = 0
	lev := 0
	if level >= 2 {
		lev = level - 2
	}
	count := 0
	for len(*stack) > 0 {
		ci := (*stack)[len(*stack)-1]
		(*stack) = (*stack)[:len(*stack)-1]
		cy := (*stack)[len(*stack)-1]
		(*stack) = (*stack)[:len(*stack)-1]
		cx := (*stack)[len(*stack)-1]
		(*stack) = (*stack)[:len(*stack)-1]
		cs := &chf.spans[ci]
		// Check if any of the neighbours already have a valid region set.
		ar := 0
		for dir := 0; dir < 4; dir++ {
			// 8 connected
			if GetCon(cs, dir) != RC_NOT_CONNECTED {
				ax := cx + GetDirOffsetX(dir)
				ay := cy + GetDirOffsetY(dir)
				ai := chf.cells[ax+ay*w].index + GetCon(cs, dir)
				if chf.areas[ai] != area {
					continue
				}
				nr := srcReg[ai]
				if (nr & RC_BORDER_REG) != 0 { // Do not take borders into account.
					continue
				}
				if nr != 0 && nr != r {
					ar = nr
					break
				}
				as := &chf.spans[ai]
				dir2 := (dir + 1) & 0x3
				if GetCon(as, dir2) != RC_NOT_CONNECTED {
					ax2 := ax + GetDirOffsetX(dir2)
					ay2 := ay + GetDirOffsetY(dir2)
					ai2 := chf.cells[ax2+ay2*w].index + GetCon(as, dir2)
					if chf.areas[ai2] != area {
						continue
					}
					nr2 := srcReg[ai2]
					if nr2 != 0 && nr2 != r {
						ar = nr2
						break
					}
				}
			}
		}
		if ar != 0 {
			srcReg[ci] = 0
			continue
		}
		count++
		// Expand neighbours.
		for dir := 0; dir < 4; dir++ {
			if GetCon(cs, dir) != RC_NOT_CONNECTED {
				ax := cx + GetDirOffsetX(dir)
				ay := cy + GetDirOffsetY(dir)
				ai := chf.cells[ax+ay*w].index + GetCon(cs, dir)
				if chf.areas[ai] != area {
					continue
				}
				if chf.dist[ai] >= lev && srcReg[ai] == 0 {
					srcReg[ai] = r
					srcDist[ai] = 0
					*stack = append(*stack, ax, ay, ai)
				}
			}
		}
	}
	return count > 0
}

func mergeAndFilterRegions(minRegionArea int, mergeRegionSize int, maxRegionId int, chf *CompactHeightfield, srcReg []int, overlaps []int) int {
	w := chf.width
	h := chf.height
	nreg := maxRegionId + 1
	regions := make([]*Region, nreg)
	for i := 0; i < nreg; i++ {
		regions[i] = &Region{id: i, ymin: 0xFFFF, connections: []int{}, floors: []int{}}
	}
	// Find edge of a region and find connections around the contour.
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			c := &chf.cells[x+y*w]
			for i, ni := c.index, c.index+c.count; i < ni; i++ {
				r := srcReg[i]
				if r == 0 || r >= nreg {
					continue
				}
				reg := regions[r]
				reg.spanCount++
				// Update floors.
				for j := c.index; j < ni; j++ {
					if i == j {
						continue
					}
					floorId := srcReg[j]
					if floorId == 0 || floorId >= nreg {
						continue
					}
					if floorId == r {
						reg.overlap = true
					}
					addUniqueFloorRegion(reg, floorId)
				}
				// Have found contour
				if len(reg.connections) > 0 {
					continue
				}
				reg.areaType = chf.areas[i]
				// Check if this cell is next to a border.
				ndir := -1
				for dir := 0; dir < 4; dir++ {
					if isSolidEdge(chf, srcReg, x, y, i, dir) {
						ndir = dir
						break
					}
				}
				if ndir != -1 {
					// The cell is at border.
					// Walk around the contour to find all the neighbours.
					walkContour(x, y, i, ndir, chf, srcReg, &reg.connections)
				}
			}
		}
	}
	// Remove too small regions.
	stack := []int{}
	trace := []int{}
	for i := 0; i < nreg; i++ {
		reg := regions[i]
		if reg.id == 0 || (reg.id&RC_BORDER_REG) != 0 {
			continue
		}
		if reg.spanCount == 0 {
			continue
		}
		if reg.visited {
			continue
		}
		// Count the total size of all the connected regions.
		// Also keep track of the regions connects to a tile border.
		connectsToBorder := false
		spanCount := 0
		stack = []int{}
		trace = []int{}
		reg.visited = true
		stack = append(stack, i)
		for len(stack) > 0 {
			// Pop
			ri := stack[len(stack)-1]
			stack = stack[:len(stack)-1]
			creg := regions[ri]
			spanCount += creg.spanCount
			trace = append(trace, ri)
			for j := 0; j < len(creg.connections); j++ {
				if (creg.connections[j] & RC_BORDER_REG) != 0 {
					connectsToBorder = true
					continue
				}
				neireg := regions[creg.connections[j]]
				if neireg.visited {
					continue
				}
				if neireg.id == 0 || (neireg.id&RC_BORDER_REG) != 0 {
					continue
				}
				// Visit
				stack = append(stack, neireg.id)
				neireg.visited = true
			}
		}
		// If the accumulated regions size is too small, remove it.
		// Do not remove areas which connect to tile borders
		// as their size cannot be estimated correctly and removing them
		// can potentially remove necessary areas.
		if spanCount < minRegionArea && !connectsToBorder {
			// Kill all visited regions.
			for j := 0; j < len(trace); j++ {
				regions[trace[j]].spanCount = 0
				regions[trace[j]].id = 0
			}
		}
	}
	// Merge too small regions to neighbour regions.
	mergeCount := 1
	for mergeCount > 0 {
		mergeCount = 0
		for i := 0; i < nreg; i++ {
			reg := regions[i]
			if reg.id == 0 || (reg.id&RC_BORDER_REG) != 0 {
				continue
			}
			if reg.overlap {
				continue
			}
			if reg.spanCount == 0 {
				continue
			}
			// Check to see if the region should be merged.
			if reg.spanCount > mergeRegionSize && isRegionConnectedToBorder(reg) {
				continue
			}
			// Small region with more than 1 connection.
			// Or region which is not connected to a border at all.
			// Find smallest neighbour region that connects to this one.
			smallest := 0xfffffff
			mergeId := reg.id
			for j := 0; j < len(reg.connections); j++ {
				if (reg.connections[j] & RC_BORDER_REG) != 0 {
					continue
				}
				mreg := regions[reg.connections[j]]
				if mreg.id == 0 || (mreg.id&RC_BORDER_REG) != 0 || mreg.overlap {
					continue
				}
				if mreg.spanCount < smallest && canMergeWithRegion(reg, mreg) && canMergeWithRegion(mreg, reg) {
					smallest = mreg.spanCount
					mergeId = mreg.id
				}
			}
			// Found new id.
			if mergeId != reg.id {
				oldId := reg.id
				target := regions[mergeId]
				// Merge neighbours.
				if mergeRegions(target, reg) {
					// Fixup regions pointing to current region.
					for j := 0; j < nreg; j++ {
						if regions[j].id == 0 || (regions[j].id&RC_BORDER_REG) != 0 {
							continue
						}
						// If another region was already merged into current region
						// change the nid of the previous region too.
						if regions[j].id == oldId {
							regions[j].id = mergeId
						}
						// Replace the current region with the new one if the
						// current regions is neighbour.
						replaceNeighbour(regions[j], oldId, mergeId)
					}
					mergeCount++
				}
			}
		}
	}
	// Compress region Ids.
	for i := 0; i < nreg; i++ {
		regions[i].remap = false
		if regions[i].id == 0 {
			continue // Skip nil regions.
		}
		if (regions[i].id & RC_BORDER_REG) != 0 {
			continue // Skip external regions.
		}
		regions[i].remap = true
	}
	regIdGen := 0
	for i := 0; i < nreg; i++ {
		if !regions[i].remap {
			continue
		}
		oldId := regions[i].id
		regIdGen++
		newId := regIdGen
		for j := i; j < nreg; j++ {
			if regions[j].id == oldId {
				regions[j].id = newId
				regions[j].remap = false
			}
		}
	}
	maxRegionId = regIdGen
	// Remap regions.
	for i := 0; i < chf.spanCount; i++ {
		if (srcReg[i] & RC_BORDER_REG) == 0 {
			srcReg[i] = regions[srcReg[i]].id
		}
	}
	// Return regions that we found to be overlapping.
	for i := 0; i < nreg; i++ {
		if regions[i].overlap {
			overlaps = append(overlaps, regions[i].id)
		}
	}
	return maxRegionId
}
func addUniqueFloorRegion(reg *Region, n int) {
	for i := 0; i < len(reg.floors); i++ {
		if reg.floors[i] == n {
			return
		}
	}
	reg.floors = append(reg.floors, n)
}
func isSolidEdge(chf *CompactHeightfield, srcReg []int, x int, y int, i int, dir int) bool {
	s := &chf.spans[i]
	r := 0
	if GetCon(s, dir) != RC_NOT_CONNECTED {
		ax := x + GetDirOffsetX(dir)
		ay := y + GetDirOffsetY(dir)
		ai := chf.cells[ax+ay*chf.width].index + GetCon(s, dir)
		r = srcReg[ai]
	}
	if r == srcReg[i] {
		return false
	}
	return true
}
func walkContour(x int, y int, i int, dir int, chf *CompactHeightfield, srcReg []int, cont *[]int) {
	startDir := dir
	starti := i
	ss := &chf.spans[i]
	curReg := 0
	if GetCon(ss, dir) != RC_NOT_CONNECTED {
		ax := x + GetDirOffsetX(dir)
		ay := y + GetDirOffsetY(dir)
		ai := chf.cells[ax+ay*chf.width].index + GetCon(ss, dir)
		curReg = srcReg[ai]
	}
	*cont = append(*cont, curReg)
	iter := 0
	for {
		iter++
		if iter >= 40000 {
			break
		}
		s := &chf.spans[i]
		if isSolidEdge(chf, srcReg, x, y, i, dir) {
			// Choose the edge corner
			r := 0
			if GetCon(s, dir) != RC_NOT_CONNECTED {
				ax := x + GetDirOffsetX(dir)
				ay := y + GetDirOffsetY(dir)
				ai := chf.cells[ax+ay*chf.width].index + GetCon(s, dir)
				r = srcReg[ai]
			}
			if r != curReg {
				curReg = r
				*cont = append(*cont, curReg)
			}
			dir = (dir + 1) & 0x3 // Rotate CW
		} else {
			ni := -1
			nx := x + GetDirOffsetX(dir)
			ny := y + GetDirOffsetY(dir)
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
	// Remove adjacent duplicates.
	if len(*cont) > 1 {
		for j := 0; j < len(*cont); {
			nj := (j + 1) % len(*cont)
			if (*cont)[j] == (*cont)[nj] {
				*cont = append((*cont)[:j], (*cont)[j+1:]...)
				//cont.remove(j);
			} else {
				j++
			}
		}
	}
}
func isRegionConnectedToBorder(reg *Region) bool {
	// Region is connected to border if
	// one of the neighbours is null id.
	for i := 0; i < len(reg.connections); i++ {
		if reg.connections[i] == 0 {
			return true
		}
	}
	return false
}
func canMergeWithRegion(rega, regb *Region) bool {
	if rega.areaType != regb.areaType {
		return false
	}
	n := 0
	for i := 0; i < len(rega.connections); i++ {
		if rega.connections[i] == regb.id {
			n++
		}
	}
	if n > 1 {
		return false
	}
	for i := 0; i < len(rega.floors); i++ {
		if rega.floors[i] == regb.id {
			return false
		}
	}
	return true
}
func mergeRegions(rega, regb *Region) bool {
	aid := rega.id
	bid := regb.id
	// Duplicate current neighbourhood.
	acon := []int{}
	acon = append(acon, rega.connections...)
	bcon := regb.connections
	// Find insertion point on A.
	insa := -1
	for i := 0; i < len(acon); i++ {
		if acon[i] == bid {
			insa = i
			break
		}
	}
	if insa == -1 {
		return false
	}
	// Find insertion point on B.
	insb := -1
	for i := 0; i < len(bcon); i++ {
		if bcon[i] == aid {
			insb = i
			break
		}
	}
	if insb == -1 {
		return false
	}
	// Merge neighbours.
	rega.connections = []int{}
	for i, ni := 0, len(acon); i < ni-1; i++ {
		rega.connections = append(rega.connections, acon[(insa+1+i)%ni])
	}
	for i, ni := 0, len(bcon); i < ni-1; i++ {
		rega.connections = append(rega.connections, bcon[(insb+1+i)%ni])
	}
	removeAdjacentNeighbours(rega)
	for j := 0; j < len(regb.floors); j++ {
		addUniqueFloorRegion(rega, regb.floors[j])
	}
	rega.spanCount += regb.spanCount
	regb.spanCount = 0
	regb.connections = []int{}
	return true
}
func removeAdjacentNeighbours(reg *Region) {
	// Remove adjacent duplicates.
	for i := 0; i < len(reg.connections) && len(reg.connections) > 1; {
		ni := (i + 1) % len(reg.connections)
		if reg.connections[i] == reg.connections[ni] {
			reg.connections = append(reg.connections[:i], reg.connections[i+1:]...)
		} else {
			i++
		}
	}
}
func replaceNeighbour(reg *Region, oldId int, newId int) {
	neiChanged := false
	for i := 0; i < len(reg.connections); i++ {
		if reg.connections[i] == oldId {
			reg.connections[i] = newId
			neiChanged = true
		}
	}
	for i := 0; i < len(reg.floors); i++ {
		if reg.floors[i] == oldId {
			reg.floors[i] = newId
		}
	}
	if neiChanged {
		removeAdjacentNeighbours(reg)
	}
}
