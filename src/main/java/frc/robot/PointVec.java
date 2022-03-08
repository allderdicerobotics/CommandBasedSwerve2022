package frc.robot;

import org.opencv.core.Point;

public class PointVec implements VectorDef<Point> {
	public Point add(Point a, Point b) {
		return new Point(a.x + b.x, a.y + b.y);
	}

	public Point scale(Point a, double n) {
		return new Point(a.x / n, a.y / n);
	}

	public Point ident() {
		return new Point(0.0, 0.0);
	}
}

