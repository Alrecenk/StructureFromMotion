/* This interface represents a minimization problem that can be solved using nonlinear optimization techniques.
 * Any continuous minimization problem that implements error and gradient can fetch a locally optimal x by using the
 * Utility class's Gradient Descent or LBFGS function. A class that also implements Hessian can use Newton's method.
 */
interface OptimizationProblem
{
	//the value to be minimized
	double error(double[] x);

	//the gradient of the value to be minimized
	double[] gradient(double[] x);

	//the Hessian of the value to be minimized
	//only requires lower triangular portion
	double[][] hessian(double[] x);

}