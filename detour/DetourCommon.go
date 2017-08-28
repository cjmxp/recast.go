package detour

import "math"

var thr float32 = sqr(1.0 / 16384.0)

func vCopy(out []float32, in []float32, i int) {
	out[0] = in[i]
	out[1] = in[i+1]
	out[2] = in[i+2]
}

func vMin(out []float32, in []float32, i int) {
	out[0] = float32(math.Min(float64(out[0]), float64(in[i])))
	out[1] = float32(math.Min(float64(out[1]), float64(in[i+1])))
	out[2] = float32(math.Min(float64(out[2]), float64(in[i+2])))
}

func vMax(out []float32, in []float32, i int) {
	out[0] = float32(math.Max(float64(out[0]), float64(in[i])))
	out[1] = float32(math.Max(float64(out[1]), float64(in[i+1])))
	out[2] = float32(math.Max(float64(out[2]), float64(in[i+2])))
}
func clamp_i(v int, min int, max int) int {
	return int(math.Max(math.Min(float64(v), float64(max)), float64(min)))
}
func clamp_f(v float32, min float32, max float32) float32 {
	return float32(math.Max(math.Min(float64(v), float64(max)), float64(min)))
}
func triArea2D(a, b, c []float32) float32 {
	abx := b[0] - a[0]
	abz := b[2] - a[2]
	acx := c[0] - a[0]
	acz := c[2] - a[2]
	return acx*abz - abx*acz
}
func nextPow2(v int) int {
	v--
	v |= v >> 1
	v |= v >> 2
	v |= v >> 4
	v |= v >> 8
	v |= v >> 16
	v++
	return v
}
func overlapQuantBounds(amin []int, amax []int, bmin []int, bmax []int) bool {
	overlap := true
	if amin[0] > bmax[0] || amax[0] < bmin[0] {
		overlap = false
	}
	if amin[1] > bmax[1] || amax[1] < bmin[1] {
		overlap = false
	}
	if amin[2] > bmax[2] || amax[2] < bmin[2] {
		overlap = false
	}
	return overlap
}

func overlapBounds(amin []float32, amax []float32, bmin []float32, bmax []float32) bool {
	overlap := true
	if amin[0] > bmax[0] || amax[0] < bmin[0] {
		overlap = false
	}
	if amin[1] > bmax[1] || amax[1] < bmin[1] {
		overlap = false
	}
	if amin[2] > bmax[2] || amax[2] < bmin[2] {
		overlap = false
	}
	return overlap
}
func vSub(v1 []float32, v2 []float32) []float32 {
	dest := make([]float32, 3)
	dest[0] = v1[0] - v2[0]
	dest[1] = v1[1] - v2[1]
	dest[2] = v1[2] - v2[2]
	return dest
}
func vAdd(v1 []float32, v2 []float32) []float32 {
	dest := make([]float32, 3)
	dest[0] = v1[0] + v2[0]
	dest[1] = v1[1] + v2[1]
	dest[2] = v1[2] + v2[2]
	return dest
}

/// Performs a linear interpolation between two vectors. (@p v1 toward @p
/// v2)
/// @param[out] dest The result vector. [(x, y, x)]
/// @param[in] v1 The starting vector.
/// @param[in] v2 The destination vector.
/// @param[in] t The interpolation factor. [Limits: 0 <= value <= 1.0]
func vLerp(verts []float32, v1 int, v2 int, t float32) []float32 {
	dest := make([]float32, 3)
	dest[0] = verts[v1+0] + (verts[v2+0]-verts[v1+0])*t
	dest[1] = verts[v1+1] + (verts[v2+1]-verts[v1+1])*t
	dest[2] = verts[v1+2] + (verts[v2+2]-verts[v1+2])*t
	return dest
}
func vLerp3(v1 []float32, v2 []float32, t float32) []float32 {
	dest := make([]float32, 3)
	dest[0] = v1[0] + (v2[0]-v1[0])*t
	dest[1] = v1[1] + (v2[1]-v1[1])*t
	dest[2] = v1[2] + (v2[2]-v1[2])*t
	return dest
}
func vDist(v1 []float32, verts []float32, i int) float32 {
	dx := verts[i] - v1[0]
	dy := verts[i+1] - v1[1]
	dz := verts[i+2] - v1[2]
	return float32(math.Sqrt(float64(dx*dx + dy*dy + dz*dz)))
}
func vperpXZ(a, b []float32) float32 {
	return a[0]*b[2] - a[2]*b[0]
}
func intersectSegSeg2D(ap []float32, aq []float32, bp []float32, bq []float32) (bool, float32, float32) {
	u := vSub(aq, ap)
	v := vSub(bq, bp)
	w := vSub(ap, bp)
	d := vperpXZ(u, v)
	if math.Abs(float64(d)) < 1e-6 {
		return false, 0, 0
	}
	s := vperpXZ(v, w) / d
	t := vperpXZ(u, w) / d
	return true, s, t
}

func distancePtPolyEdgesSqr(pt []float32, verts []float32, nverts int, ed []float32, et []float32) bool {
	// TODO: Replace pnpoly with triArea2D tests?
	c := false
	for i, j := 0, nverts-1; i < nverts; j, i = i, i+1 {
		vi := i * 3
		vj := j * 3
		if ((verts[vi+2] > pt[2]) != (verts[vj+2] > pt[2])) && (pt[0] < (verts[vj+0]-verts[vi+0])*(pt[2]-verts[vi+2])/(verts[vj+2]-verts[vi+2])+verts[vi+0]) {
			c = !c
		}
		ed[j], et[j] = distancePtSegSqr2D(pt, verts, vj, vi)
	}
	return c
}
func distancePtSegSqr2D3(pt []float32, p []float32, q []float32) (float32, float32) {
	pqx := q[0] - p[0]
	pqz := q[2] - p[2]
	dx := pt[0] - p[0]
	dz := pt[2] - p[2]
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
	dx = p[0] + t*pqx - pt[0]
	dz = p[2] + t*pqz - pt[2]
	return dx*dx + dz*dz, t
}
func distancePtSegSqr2D(pt []float32, verts []float32, p int, q int) (float32, float32) {
	pqx := verts[q+0] - verts[p+0]
	pqz := verts[q+2] - verts[p+2]
	dx := pt[0] - verts[p+0]
	dz := pt[2] - verts[p+2]
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
	dx = verts[p+0] + t*pqx - pt[0]
	dz = verts[p+2] + t*pqz - pt[2]
	return dx*dx + dz*dz, t
}

func closestHeightPointTriangle(p []float32, a []float32, b []float32, c []float32) (bool, float32) {
	v0 := vSub(c, a)
	v1 := vSub(b, a)
	v2 := vSub(p, a)

	dot00 := vDot2D(v0, v0)
	dot01 := vDot2D(v0, v1)
	dot02 := vDot2D(v0, v2)
	dot11 := vDot2D(v1, v1)
	dot12 := vDot2D(v1, v2)

	// Compute barycentric coordinates
	invDenom := 1.0 / (dot00*dot11 - dot01*dot01)
	u := (dot11*dot02 - dot01*dot12) * invDenom
	v := (dot00*dot12 - dot01*dot02) * invDenom

	// The (sloppy) epsilon is needed to allow to get height of points which
	// are interpolated along the edges of the triangles.

	// If point lies inside the triangle, return interpolated ycoord.
	if u >= -EPS && v >= -EPS && (u+v) <= 1+EPS {
		h := a[1] + v0[1]*u + v1[1]*v
		return true, h
	}
	return false, 0
}

/// Derives the dot product of two vectors on the xz-plane. (@p u . @p v)
/// @param[in] u A vector [(x, y, z)]
/// @param[in] v A vector [(x, y, z)]
/// @return The dot product on the xz-plane.
///
/// The vectors are projected onto the xz-plane, so the y-values are
/// ignored.
func vDot2D(u []float32, v []float32) float32 {
	return u[0]*v[0] + u[2]*v[2]
}

/// Derives the square of the scalar length of the vector. (len * len)
/// @param[in] v The vector. [(x, y, z)]
/// @return The square of the scalar length of the vector.
func vLenSqr(v []float32) float32 {
	return v[0]*v[0] + v[1]*v[1] + v[2]*v[2]
}
func vEqual(p0, p1 []float32) bool {
	d := vDistSqr(p0, p1)
	return d < thr
}
func vDistSqr(v1, v2 []float32) float32 {
	dx := v2[0] - v1[0]
	dy := v2[1] - v1[1]
	dz := v2[2] - v1[2]
	return dx*dx + dy*dy + dz*dz
}
func oppositeTile(side int) int {
	return (side + 4) & 0x7
}
func sqr(a float32) float32 {
	return a * a
}
