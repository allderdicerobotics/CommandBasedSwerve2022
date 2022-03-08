package frc.robot;

import java.util.stream.*;

public interface VectorDef<T> {
	public T add(T a, T b);
	public T scale(T a, double n);
	public T ident();

	public T mean(Stream<T> stream) {
		return this.scale(
			stream.reduce(this.ident(), this::add),
			(double) (1 / stream.count())
		);
	}
}

