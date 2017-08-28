package recast

import "math"

func copy3(out []float32, n int, in []float32, m int) {
	out[n] = in[m]
	out[n+1] = in[m+1]
	out[n+2] = in[m+2]
}

func min(a []float32, b []float32, i int) {
	a[0] = float32(math.Min(float64(a[0]), float64(b[i+0])))
	a[1] = float32(math.Min(float64(a[1]), float64(b[i+1])))
	a[2] = float32(math.Min(float64(a[2]), float64(b[i+2])))
}
func max(a []float32, b []float32, i int) {
	a[0] = float32(math.Max(float64(a[0]), float64(b[i+0])))
	a[1] = float32(math.Max(float64(a[1]), float64(b[i+1])))
	a[2] = float32(math.Max(float64(a[2]), float64(b[i+2])))
}
func add(e0 []float32, a []float32, verts []float32, i int) {
	e0[0] = a[0] + verts[i]
	e0[1] = a[1] + verts[i+1]
	e0[2] = a[2] + verts[i+2]
}
func sub(e0 []float32, verts []float32, i int, j int) {
	e0[0] = verts[i] - verts[j]
	e0[1] = verts[i+1] - verts[j+1]
	e0[2] = verts[i+2] - verts[j+2]
}
func sub3(e0 []float32, i []float32, verts []float32, j int) {
	e0[0] = i[0] - verts[j]
	e0[1] = i[1] - verts[j+1]
	e0[2] = i[2] - verts[j+2]
}
func cross(dest, v1, v2 []float32) {
	dest[0] = v1[1]*v2[2] - v1[2]*v2[1]
	dest[1] = v1[2]*v2[0] - v1[0]*v2[2]
	dest[2] = v1[0]*v2[1] - v1[1]*v2[0]
}

func normalize(v []float32) {
	d := float32(1.0 / math.Sqrt(float64(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])))
	v[0] *= d
	v[1] *= d
	v[2] *= d
}
