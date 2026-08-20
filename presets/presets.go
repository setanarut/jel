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
		MassPerPoint:        0.5,
		ShapeMatchStiffness: 150,
		ShapeMatchDamping:   30,
		ShapeMatching:       true,
	}
}

func CarTire() jel.PressureBodyOptions {
	return jel.PressureBodyOptions{
		SpringBodyOptions: jel.SpringBodyOptions{
			// Stiffness'i ÇOK YÜKSEK tutuyoruz ki dış yüzey beton gibi/çelik kuşaklı lastik gibi olsun, esnemesin.
			SpringMat: jel.SpringMat{Stiffness: 1500.0, Damping: 40.0},

			// Arabanın gövdesini (3.0 * 9 nokta = 27 birim) taşıyabilmesi için kütlesi 1.0 kalabilir.
			MassPerPoint: 1.0,

			ShapeMatching:       true,
			ShapeMatchStiffness: 1500,
			ShapeMatchDamping:   50,
		},
		// Hacim küçük olduğu için 20-30 arası bir basınç tekerleği şişkin tutmaya fazlasıyla yetecektir.
		GasPressure: 1.0,
	}
}
