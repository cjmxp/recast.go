package recast

import "math"

func clamp_i(v, min, max int) int {
	return int(math.Max(math.Min(float64(max), float64(v)), float64(min)))
}
func GetDirOffsetX(dir int) int {
	offset := []int{-1, 0, 1, 0}
	return offset[dir&0x03]
}
func GetDirOffsetY(dir int) int {
	offset := []int{0, 1, 0, -1}
	return offset[dir&0x03]
}
func SetCon(s *CompactSpan, dir int, i int) {
	shift := uint(dir * 6)
	con := s.con
	s.con = (con & ^(0x3f << shift)) | ((i & 0x3f) << shift)
}
func GetCon(s *CompactSpan, dir int) int {
	shift := uint8(dir * 6)
	return (s.con >> shift) & 0x3f
}
func rcGetDirForOffset(x int, y int) int {
	dirs := []int{3, 0, -1, 2, 1}
	return dirs[((y+1)<<1)+x]
}
