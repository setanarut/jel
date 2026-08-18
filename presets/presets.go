package presets

import "github.com/setanarut/jel"

var (
	RubberBand = jel.SpringMat{100.0, 1.0} // Paket Lastiği
	Jelly      = jel.SpringMat{25.0, 0.2}  // Yumuşak Jöle
	SteelWire  = jel.SpringMat{500.0, 5.0} // Sert Çelik
)

func Baloon() jel.PressureBodyOptions {
	return jel.PressureBodyOptions{
		SpringBodyOptions: jel.SpringBodyOptions{
			SpringMat:           jel.SpringMat{80, 4},
			MassPerPoint:        0.1,
			ShapeMatchStiffness: 30,
			ShapeMatchDamping:   0.5,
			ShapeMatching:       true,
		},
		GasPressure: 3,
	}

}

func Jell() jel.SpringBodyOptions {
	return jel.SpringBodyOptions{
		SpringMat:           jel.SpringMat{15, 3},
		MassPerPoint:        0.3,
		ShapeMatchStiffness: 150,
		ShapeMatchDamping:   30,
		ShapeMatching:       true,
	}
}
