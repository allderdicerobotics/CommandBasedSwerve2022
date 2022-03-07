package frc.robot;

public class MapValues extends Function<Double, Double> {
	private final double fromS = 0.0;
	private final double fromE = 1.0;
	private final double toS = 0.0;
	private final double toE = 1.0;

	public MapValues(double fromS, double fromE, double toS, double toE) {
		this.fromS = fromS;
		this.fromE = fromE;
		this.toS = toS;
		this.toE = toE;
	}

	public MapValues() {}

	public MapValues to(double fromS, double fromE) {
		this.fromS = fromS;
		this.fromE = fromE;
		return this;
	}

	public MapValues from(double toS, double toE) {
		this.toS = toS;
		this.toE = toE;
		return this;
	}
}
