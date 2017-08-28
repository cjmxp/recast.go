package recast

import "math"

func RasterizeTriangles(verts []float32, tris []int, areas []int, nt int, solid *Heightfield, flagMergeThr int) {
	ics := 1.0 / solid.cs
	ich := 1.0 / solid.ch
	// Rasterize triangles.
	for i := 0; i < nt; i++ {
		v0 := tris[i*3+0]
		v1 := tris[i*3+1]
		v2 := tris[i*3+2]
		// Rasterize.
		rasterizeTri(verts, v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr)
	}
}

func rasterizeTri(verts []float32, v0 int, v1 int, v2 int, area int, hf *Heightfield, bmin []float32, bmax []float32, cs float32, ics float32, ich float32, flagMergeThr int) {
	w := hf.width
	h := hf.height
	tmin := make([]float32, 3)
	tmax := make([]float32, 3)
	by := bmax[1] - bmin[1]
	// Calculate the bounding box of the triangle.
	copy3(tmin, 0, verts, v0*3)
	copy3(tmax, 0, verts, v0*3)
	min(tmin, verts, v1*3)
	min(tmin, verts, v2*3)
	max(tmax, verts, v1*3)
	max(tmax, verts, v2*3)
	// If the triangle does not touch the bbox of the heightfield, skip the triagle.
	if !overlapBounds(bmin, bmax, tmin, tmax) {
		return
	}
	// Calculate the footprint of the triangle on the grid's y-axis
	y0 := int((tmin[2] - bmin[2]) * ics)
	y1 := int((tmax[2] - bmin[2]) * ics)
	y0 = clamp_i(y0, 0, h-1)
	y1 = clamp_i(y1, 0, h-1)
	// Clip the triangle into all grid cells it touches.
	buf := make([]float32, 7*3*4)
	in := 0
	inrow := 7 * 3
	p1 := inrow + 7*3
	p2 := p1 + 7*3
	copy3(buf, 0, verts, v0*3)
	copy3(buf, 3, verts, v1*3)
	copy3(buf, 6, verts, v2*3)
	nvrow, nvIn := 3, 3
	for y := y0; y <= y1; y++ {
		// Clip polygon to row. Store the remaining polygon as well
		cz := bmin[2] + float32(y)*cs
		nvrowin := dividePoly(buf, in, nvIn, inrow, p1, cz+cs, 2)
		nvrow = nvrowin[0]
		nvIn = nvrowin[1]
		in, p1 = p1, in
		if nvrow < 3 {
			continue
		}
		// find the horizontal bounds in the row
		minX, maxX := buf[inrow], buf[inrow]
		for i := 1; i < nvrow; i++ {
			if minX > buf[inrow+i*3] {
				minX = buf[inrow+i*3]
			}
			if maxX < buf[inrow+i*3] {
				maxX = buf[inrow+i*3]
			}
		}
		x0 := int((minX - bmin[0]) * ics)
		x1 := int((maxX - bmin[0]) * ics)
		x0 = clamp_i(x0, 0, w-1)
		x1 = clamp_i(x1, 0, w-1)

		nv, nv2 := nvrow, nvrow
		for x := x0; x <= x1; x++ {
			// Clip polygon to column. store the remaining polygon as well
			cx := bmin[0] + float32(x)*cs
			nvnv2 := dividePoly(buf, inrow, nv2, p1, p2, cx+cs, 0)
			nv = nvnv2[0]
			nv2 = nvnv2[1]
			inrow, p2 = p2, inrow
			if nv < 3 {
				continue
			}
			// Calculate min and max of the span.
			smin, smax := buf[p1+1], buf[p1+1]
			for i := 1; i < nv; i++ {
				smin = float32(math.Min(float64(smin), float64(buf[p1+i*3+1])))
				smax = float32(math.Max(float64(smax), float64(buf[p1+i*3+1])))
			}
			smin -= bmin[1]
			smax -= bmin[1]
			// Skip the span if it is outside the heightfield bbox
			if smax < 0.0 {
				continue
			}
			if smin > by {
				continue
			}
			// Clamp the span to the heightfield bbox.
			if smin < 0.0 {
				smin = 0
			}
			if smax > by {
				smax = by
			}
			// Snap the span to the heightfield height grid.
			ismin := clamp_i(int(math.Floor(float64(smin*ich))), 0, RC_SPAN_MAX_HEIGHT)
			ismax := clamp_i(int(math.Ceil(float64(smax*ich))), ismin+1, RC_SPAN_MAX_HEIGHT)
			addSpan(hf, x, y, ismin, ismax, area, flagMergeThr)
		}
	}
}
func overlapBounds(amin, amax, bmin, bmax []float32) bool {
	overlap := true
	if amin[0] > bmax[0] || amax[0] < bmin[0] {
		overlap = false
	} else if amin[1] > bmax[1] || amax[1] < bmin[1] {
		overlap = false
	} else if amin[2] > bmax[2] || amax[2] < bmin[2] {
		overlap = false
	}
	return overlap
}
func dividePoly(buf []float32, in int, nin int, out1 int, out2 int, x float32, axis int) []int {
	d := make([]float32, 12)
	for i := 0; i < nin; i++ {
		d[i] = x - buf[in+i*3+axis]
	}
	m, n := 0, 0
	for i, j := 0, nin-1; i < nin; j, i = i, i+1 {
		ina := d[j] >= 0
		inb := d[i] >= 0
		if ina != inb {
			s := d[j] / (d[j] - d[i])
			buf[out1+m*3+0] = buf[in+j*3+0] + (buf[in+i*3+0]-buf[in+j*3+0])*s
			buf[out1+m*3+1] = buf[in+j*3+1] + (buf[in+i*3+1]-buf[in+j*3+1])*s
			buf[out1+m*3+2] = buf[in+j*3+2] + (buf[in+i*3+2]-buf[in+j*3+2])*s
			copy3(buf, out2+n*3, buf, out1+m*3)
			m++
			n++
			// add the i'th point to the right polygon. Do NOT add points that are on the dividing line
			// since these were already added above
			if d[i] > 0 {
				copy3(buf, out1+m*3, buf, in+i*3)
				m++
			} else if d[i] < 0 {
				copy3(buf, out2+n*3, buf, in+i*3)
				n++
			}
		} else { // same side
			// add the i'th point to the right polygon. Addition is done even for points on the dividing line
			if d[i] >= 0 {
				copy3(buf, out1+m*3, buf, in+i*3)
				m++
				if d[i] != 0 {
					continue
				}
			}
			copy3(buf, out2+n*3, buf, in+i*3)
			n++
		}
	}
	return []int{m, n}
}

func addSpan(hf *Heightfield, x, y, smin, smax, area, flagMergeThr int) {
	idx := x + y*hf.width
	s := &Span{}
	s.smin = smin
	s.smax = smax
	s.area = area
	s.next = nil
	// Empty cell, add the first span.
	if hf.spans[idx] == nil {
		hf.spans[idx] = s
		return
	}
	var prev *Span = nil
	cur := hf.spans[idx]
	// Insert and merge spans.
	for cur != nil {
		if cur.smin > s.smax {
			// Current span is further than the new span, break.
			break
		} else if cur.smax < s.smin {
			// Current span is before the new span advance.
			prev = cur
			cur = cur.next
		} else {
			// Merge spans.
			if cur.smin < s.smin {
				s.smin = cur.smin
			}
			if cur.smax > s.smax {
				s.smax = cur.smax
			}
			// Merge flags.
			if int(math.Abs(float64(s.smax-cur.smax))) <= flagMergeThr {
				s.area = int(math.Max(float64(s.area), float64(cur.area)))
			}
			// Remove current span.
			next := cur.next
			if prev != nil {
				prev.next = next
			} else {
				hf.spans[idx] = next
			}
			cur = next
		}
	}
	// Insert new span.
	if prev != nil {
		s.next = prev.next
		prev.next = s
	} else {
		s.next = hf.spans[idx]
		hf.spans[idx] = s
	}
}
