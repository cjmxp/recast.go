package recast

import (
	"bufio"
	"os"
	"strconv"
	"strings"
)

type ObjImporter struct {
	vertexPositions []float32
	meshFaces       []int
}

func (this *ObjImporter) Load(path string) *InputGeom {
	if file, err := os.Open(path); err == nil {
		reader := bufio.NewReader(file)
		for {
			if buff, _, e := reader.ReadLine(); e == nil {
				this.readLine([]byte(strings.Trim(string(buff), " ")))
			} else {
				break
			}
		}
		file.Close()
	} else {
		panic(err.Error())
	}
	ig := &InputGeom{}
	ig.Init(this.vertexPositions, this.meshFaces)
	return ig
}

func (this *ObjImporter) readLine(buff []byte) {
	if len(buff) == 0 {
		return
	}
	if buff[0] == 'v' {
		this.readVector(string(buff))
	} else if buff[0] == 'f' {
		this.readFace(string(buff))
	}
}
func (this *ObjImporter) readVector(line string) {
	if line[1] != ' ' {
		return
	}
	v := strings.Split(line, " ")
	if len(v) < 4 {
		panic("Invalid vector, expected 3 coordinates, found " + strconv.FormatInt(int64(len(v)), 10))
	}
	x, _ := strconv.ParseFloat(v[1], 32)
	y, _ := strconv.ParseFloat(v[2], 32)
	z, _ := strconv.ParseFloat(v[3], 32)
	this.vertexPositions = append(this.vertexPositions, float32(x), float32(y), float32(z))
}

func (this *ObjImporter) readFace(line string) {
	v := strings.Split(line, " ")
	if len(v) < 4 {
		panic("Invalid number of face vertices: 3 coordinates expected, found " + strconv.FormatInt(int64(len(v)), 10))
	}
	for j := 0; j < len(v)-3; j++ {
		this.meshFaces = append(this.meshFaces, this.readFaceVertex(v[1]))
		for i := 0; i < 2; i++ {
			this.meshFaces = append(this.meshFaces, this.readFaceVertex(v[2+j+i]))
		}
	}
}

func (this *ObjImporter) readFaceVertex(face string) int {
	v := strings.Split(face, "/")
	i, _ := strconv.ParseInt(v[0], 10, 32)
	return this.getIndex(int(i), len(this.vertexPositions))
}

func (this *ObjImporter) getIndex(posi, size int) int {
	if posi > 0 {
		posi--
	} else if posi < 0 {
		posi = size + posi
	} else {
		panic("0 vertex index")
	}
	return posi
}
