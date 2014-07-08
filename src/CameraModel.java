/* This class uses corresponding points from 2 images to determine the relative location and orientation of the cameras that took the pictures
 * That information can then be used to create a 3D reconstruction of corresponding points across those two images.
 * 
 * The best way to generate a CameraModel is to use the static method RandomSampleCalibration, 
 * which repeatedly calibrates using random initialization and a subset of the calibration points making it resistant to outliers and local minima
 * 
 * The method employed here a direct minimization of reconstruction error with penalties to resist skewing and scaling in the camera matrices.
 * 
 * Model is  A [ xt yt t]_i ^T *r_i = T + B [us vs s]_i*s_i for all points (i)
 * where A and B are the camera matrices (a combination of extrinsic and intrinsic parameters)
 * T is the vector from camera A to camera B, (x,y) are the points on A and (u,v) are the points on B
 * with s and t being the positions along the lines that project into those pixels
 * intrinsic parameters are given, B is fixed during optimization and T is fixed to a scale of 1 to prevent arbitrary scaling
 * 
 * The objectvice function being minimized by L-BFGS is
 * Error is  sum (  ( A *[ x y 1]^T *r) +T - (B*[ u v 1 ]^T * s ) )^2
 *	 given A = [ X Y Z ]^T
 *	 + lamda * [  ( X^2 - w^2)^2 + ( Y^2 - h^2)^2 + ( Z^2 - d^2)^2 + ( T^2 - 1)^2 ]  //no scale in translation or A matrix (match intrinsic parameters)
 *	 + gamma * [ (X dot Y)^2 + (X dot Z)^2 + (Y dot Z)^2 ] // no skew in A matrix so it has to be a rotation
 */ 


public class CameraModel implements OptimizationProblem{

	public double w,h,d ;//width, height, and depth of frame (intrinsic parameters
	//Model is  A [ xt yt t]_i ^T *r_i = T + B [us vs s]_i*s_i for all points( i)
	public double  T[], A[][], B[][] ;

	//Error is  sum (  ( A *[ x y 1]^T *r) +T - (B*[ u v 1 ]^T * s ) )^2
	// given A = [ X Y Z ]^T
	// + lamda * [  ( X^2 - w^2)^2 + ( Y^2 - h^2)^2 + ( Z^2 - d^2)^2 + ( T^2 - 1)^2 ]  //no scale in A matrix (match intrinsic parameters)
	// + gamma * [ (X dot Y)^2 + (X dot Z)^2 + (Y dot Z)^2 ] // no skew in A matrix so it has to be a rotation
	double lamda, gamma ;   

	//matches should be in an image space where x and y both range from -0.5 to 0.5 across the image
	double matches[][][] ;//first index is which match, second is which image, third is {x,y}
	double s[], r[] ; //


	//initalizes a cameras model object with intrinsic parameters but does not calibrate it
	public CameraModel(double widthperdepth, double heightperdepth, double scalepenalty, double skewpenalty){
		w = widthperdepth ;
		h = heightperdepth ;
		d = 1 ;
		lamda = scalepenalty ;
		gamma = skewpenalty ;
		T = Utility.scaleTo(new double[]{Math.random()-0.5,Math.random()-0.5,Math.random()-0.5},1) ;
		A = new double[][]{
				new double[]{ w, 0 , 0},
				new double[]{ 0, h , 0},
				new double[]{ 0, 0 , d}
		} ;
		B = new double[][]{
				new double[]{ w, 0 , 0},
				new double[]{ 0, h , 0},
				new double[]{ 0, 0 , d}
		} ;
	}

	//adjusts the model to match the corresponding points given in image coordinates 
	public void calibrate(double correspondence[][][], int imagewidth, int imageheight){
		//first scale the correspondences to -0.5 to 0.5 range
		//and put the in the matches variable to be used for optimization
		//along with the starting t and s values
		matches = new double[correspondence.length][2][2] ;
		r = new double[correspondence.length] ;
		s = new double[correspondence.length] ;
		for(int k=0;k<correspondence.length;k++){
			//map points into -0.5 to 0.5 range
			matches[k][0][0] = (correspondence[k][0][0]/imagewidth)-0.5 ;
			matches[k][0][1] = (correspondence[k][0][1]/imageheight)-0.5 ;
			matches[k][1][0] = (correspondence[k][1][0]/imagewidth)-0.5 ;
			matches[k][1][1] = (correspondence[k][1][1]/imageheight)-0.5 ;
			//set initial positions of points to be kind of far out in front
			r[k] = 10 ;
			s[k] = 10 ;
		}
		double w[] = getVector(A, T, r, s) ;
		//this really doesn't take long so we can shoot for very high accuracy
		w = Utility.optimumbyLBFGS(this, w, 5, 10000, 2000, 0.000000001, 0.000000001) ;

		//convert compound parameter back into its component parts
		A = getA(w) ;
		T = getT(w) ;
		r = getr(w);
		s = gets(w) ;

	}


	//gets a 3D point given correspondences on two images with the given dimensions (used to calibrate this model)
	public double[] get3Dpoint(double correspondence[][], int imagewidth, int imageheight){
		double[][] match = new double[2][3] ;
		match[0][0] = (correspondence[0][0]/imagewidth)-0.5 ;
		match[0][1] = (correspondence[0][1]/imageheight)-0.5 ;
		match[0][2] = 1 ;
		match[1][0] = (correspondence[1][0]/imagewidth)-0.5 ;
		match[1][1] = (correspondence[1][1]/imageheight)-0.5 ;
		match[1][2] = 1 ;
		return get3Dpoint(match) ;
	}

	//gets a 3D point's alignment error given correspondences on two images with the given dimensions (used to calibrate this model)
	public double get3DError(double correspondence[][], int imagewidth, int imageheight){
		double[][] match = new double[2][3] ;
		match[0][0] = (correspondence[0][0]/imagewidth)-0.5 ;
		match[0][1] = (correspondence[0][1]/imageheight)-0.5 ;
		match[0][2] = 1 ;

		match[1][0] = (correspondence[1][0]/imagewidth)-0.5 ;
		match[1][1] = (correspondence[1][1]/imageheight)-0.5 ;
		match[1][2] = 1 ;
		return get3Derror(match) ;
	}


	//given a match (in the -0.5 to 0.5 range) returns the 3D point
	public double[] get3Dpoint(double match[][]){
		return closestIntersection(new double[3], Utility.multiply(A,match[0]), T,  Utility.multiply(B,match[1]) );
	}

	//given a match (in the -0.5 to 0.5 range) returns the calibration error of the 3D point
	public double get3Derror(double match[][]){
		return intersectionDistance(new double[3], Utility.multiply(A,match[0]), T,  Utility.multiply(B,match[1]) );
	}

	//returns the point closest to intersection of two 3D lines defined parametrically as p1 + t*v1 and p2 + s *v2
	public double[] closestIntersection(double p1[], double v1[], double p2[], double v2[]){
		double w0[] = Utility.subtract(p1, p2);
		double a = Utility.dot(v1,v1),
				b = Utility.dot(v1,v2),
				c = Utility.dot(v2,v2),
				d = Utility.dot(v1,w0),
				e = Utility.dot(v2, w0) ;
		double f = a*c-b*b ;
		double t = (b*e-c*d)/f ;
		double s = (a*e-b*d)/f ;
		double x1[] = new double[]{p1[0] + t * v1[0], p1[1] + t * v1[1], p1[2] + t * v1[2]} ;
		double x2[] = new double[]{p2[0] + s * v2[0], p2[1] + s * v2[1], p2[2] + s * v2[2]} ;	
		return new double[]{ (x1[0] + x2[0])*0.5, (x1[1] + x2[1])*0.5, (x1[2] + x2[2])*0.5} ;
	}


	//returns the shortest distance between two line defined parametrically as p1 + t*v1 and p2 + s *v2
	public double intersectionDistance(double p1[], double v1[], double p2[], double v2[]){
		double w0[] = Utility.subtract(p1, p2);
		double a = Utility.dot(v1,v1),
				b = Utility.dot(v1,v2),
				c = Utility.dot(v2,v2),
				d = Utility.dot(v1,w0),
				e = Utility.dot(v2, w0) ;
		double f = a*c-b*b ;
		double t = (b*e-c*d)/f ;
		double s = (a*e-b*d)/f ;
		double x1[] = new double[]{p1[0] + t * v1[0], p1[1] + t * v1[1], p1[2] + t * v1[2]} ;
		double x2[] = new double[]{p2[0] + s * v2[0], p2[1] + s * v2[1], p2[2] + s * v2[2]} ;	
		return Math.sqrt((x1[0]-x2[0])*(x1[0]-x2[0])+(x1[1]-x2[1])*(x1[1]-x2[1])+(x1[2]-x2[2])*(x1[2]-x2[2]));
	}





	//turns A, T, r, and s into a single vector for use with optimization methods
	public static double[] getVector(double A[][], double T[], double r[], double s[]){
		double vector[] = new double[9+3+r.length+s.length] ;
		//put A in vector
		vector[0] = A[0][0] ;
		vector[1] = A[0][1] ;
		vector[2] = A[0][2] ;
		vector[3] = A[1][0] ;
		vector[4] = A[1][1] ;
		vector[5] = A[1][2] ;
		vector[6] = A[2][0] ;
		vector[7] = A[2][1] ;
		vector[8] = A[2][2] ;
		//put T in vector
		vector[9] = T[0] ;
		vector[10] = T[1] ;
		vector[11] = T[2] ;

		int j = 12 ;
		//put r in vector
		for(int k=0;k<r.length;k++){
			vector[j] = r[k] ;
			j++;
		}
		//put s in vector
		for(int k=0;k<s.length;k++){
			vector[j] = s[k] ;
			j++;
		}
		return vector ;
	}

	//get A as a matrix from the single vector used for optimization
	public double[][] getA(double x[]){
		return new double[][]{
				new double[]{ x[0], x[1] , x[2]},
				new double[]{ x[3], x[4] , x[5]},
				new double[]{ x[6], x[7] , x[8]}
		} ;
	}

	//gets T from the single vector used for optimization
	public double[] getT(double x[]){
		return new double[]{ x[9], x[10], x[11]} ;
	}

	//gets r from the single vector used for optimization
	public double[] getr(double x[]){
		double r[] = new double[matches.length] ;
		for(int k=0;k<r.length;k++){
			r[k] = x[12+k];
		}
		return r ;
	}

	//gets s from the single vector used for optimization
	public double[] gets(double x[]){
		double s[] = new double[matches.length] ;
		for(int k=0;k<s.length;k++){
			s[k] = x[12 + s.length + k];
		}
		return s ;
	}


	//returns the value of the objective function for the given setting of parameters
	public double error(double[] x) {
		double A[][] = getA(x);
		double T[] = getT(x) ;
		double r[] = getr(x) ;
		double s[] = gets(x) ;
		double error = 0 ;

		for(int i=0;i<matches.length;i++){
			//get 3D locations of the point on each image
			double a[] = Utility.multiply(A, new double[]{ matches[i][0][0]*r[i], matches[i][0][1]*r[i], r[i]}); 
			double b[] = Utility.multiply(B, new double[]{ matches[i][1][0]*s[i], matches[i][1][1]*s[i], s[i]}); 
			//add squared error to error
			error += Utility.distancesquared(a,Utility.add(b,T)) ;
		}

		double X[] = new double[] {A[0][0], A[1][0], A[2][0]} ;
		double Y[] = new double[] {A[0][1], A[1][1], A[2][1]} ;
		double Z[] = new double[] {A[0][2], A[1][2], A[2][2]} ;

		//penalize incorrect scaling
		double err = (Utility.dot(X,X)-w*w);
		error += lamda*err*err ;
		err = (Utility.dot(Y,Y)-h*h);
		error += lamda*err*err ;
		err = (Utility.dot(Z,Z)-d*d);
		error += lamda*err*err ;
		err = (Utility.dot(T,T)-1);
		error += lamda*err*err ;

		//penalize skewing
		err = Utility.dot(X,Y) ;
		error += gamma*err*err ;
		err = Utility.dot(X,Z) ;
		error += gamma*err*err ;
		err = Utility.dot(Z,Y) ;
		error += gamma*err*err ;

		return error;
	}

	//returns the error of the current calibration settings
	public double error(){
		return error(getVector(A,T,r,s)) ;
	}


	//returns the gradient of the objective function for the given setting of parameters
	public double[] gradient(double[] x) {
		//convert from vector format to matrix A and vector T,r, and s
		double A[][] = getA(x);
		double T[] = getT(x) ;
		double r[] = getr(x) ;
		double s[] = gets(x) ;
		//gradients
		double dT[] = new double[3] ;
		double dA[][] = new double[3][3] ;
		double dr[] = new double[r.length];
		double ds[] = new double[s.length] ;



		for(int i=0;i<matches.length;i++){
			//vectors from camera centers toward pixel
			//a = A [x y 1]^T
			double a[] = Utility.multiply(A, new double[]{ matches[i][0][0], matches[i][0][1], 1}) ;
			//b = B [u v 1]^T
			double b[] = Utility.multiply(B, new double[]{ matches[i][1][0], matches[i][1][1], 1}); 
			//diff = A *[x y 1]^T* r - T - B [u v 1]^T * s
			double diff[] = new double[]{ 
					a[0]*r[i] - T[0] - b[0]*s[i],
					a[1]*r[i] - T[1] - b[1]*s[i],
					a[2]*r[i] - T[2] - b[2]*s[i]
			} ;

			//set derivatives for length along lines for this match 
			dr[i] =  2 * Utility.dot(a,diff) ;
			ds[i] = -2 * Utility.dot(b,diff) ;

			//add derivatives for translation vector from A to B that would help for point i

			dT = Utility.add(dT, Utility.scale(diff, -2)) ;

			//add derivatives to A 

			dA[0][0] += 2*diff[0] * matches[i][0][0] *r[i] ;
			dA[1][0] += 2*diff[1] * matches[i][0][0] *r[i] ;
			dA[2][0] += 2*diff[2] * matches[i][0][0] *r[i] ;

			dA[0][1] += 2*diff[0] * matches[i][0][1] *r[i] ;
			dA[1][1] += 2*diff[1] * matches[i][0][1] *r[i] ;
			dA[2][1] += 2*diff[2] * matches[i][0][1] *r[i] ;

			dA[0][2] += 2*diff[0] * 1 *r[i] ;
			dA[1][2] += 2*diff[1] * 1 *r[i] ;
			dA[2][2] += 2*diff[2] * 1 *r[i] ;

		}

		double X[] = new double[] {A[0][0], A[1][0], A[2][0]} ;
		double Y[] = new double[] {A[0][1], A[1][1], A[2][1]} ;
		double Z[] = new double[] {A[0][2], A[1][2], A[2][2]} ;

		//gradient of penalties for incorrect scaling
		double dX[] = Utility.scale(X,4*lamda*(Utility.dot(X,X)-w*w)) ;
		double dY[] = Utility.scale(Y,4*lamda*(Utility.dot(Y,Y)-h*h)) ;
		double dZ[] = Utility.scale(Z,4*lamda*(Utility.dot(Z,Z)-d*d)) ;
		dT = Utility.add(dT, Utility.scale(T,4*lamda*(Utility.dot(T,T)-1))) ;

		//gradient of penalties from skew
		double XY = Utility.dot(X,Y) ;
		double XZ = Utility.dot(X,Z) ;
		double ZY = Utility.dot(Z,Y) ;
		dX = Utility.add(dX, Utility.scale(Y, 2*gamma * XY)) ;
		dX = Utility.add(dX, Utility.scale(Z, 2*gamma * XZ)) ;

		dY = Utility.add(dY, Utility.scale(X, 2*gamma * XY)) ;
		dY = Utility.add(dY, Utility.scale(Z, 2*gamma * ZY)) ;

		dZ = Utility.add(dZ, Utility.scale(Y, 2*gamma * ZY)) ;
		dZ = Utility.add(dZ, Utility.scale(X, 2*gamma * XZ)) ;

		//Add A gradients from penalties to gradients from point error
		dA[0][0] += dX[0] ;
		dA[1][0] += dX[1] ;
		dA[2][0] += dX[2] ;

		dA[0][1] += dY[0] ;
		dA[1][1] += dY[1] ;
		dA[2][1] += dY[2] ;

		dA[0][2] += dZ[0] ;
		dA[1][2] += dZ[1] ;
		dA[2][2] += dZ[2] ;

		//append them back into a single vector format
		return getVector(dA, dT, dr, ds);
	}


	public double[][] hessian(double[] x) {
		//Auto-generated method stub
		System.err.println("Hessian not defined for Camera Model. Use L-BFGS instead of Newton's method.");
		return null;
	}

	//Calibrates repeatedly with random initialization and random sampling of the corresponding points and returns the lowest error model
	//this method returns the model that gets the lowest error on only the points used to define it
	//thus sample size should be > 5 (about 10 is good)
	//tries in the range of 10 to 50 is good ( need to luck into a good initialization position)
	//Pixels are assumed to be perfectly square so widthperdepth (the width of the image in 3D space divided by depth) is the only intrinsic parameter necessary
	public static CameraModel RandomSampleCalibration(double widthperdepth, int imagewidth, int imageheight, double correspondence[][][], int tries, int samplesize){
		CameraModel bestmodel = null;
		double besterror = Double.MAX_VALUE ;
		for(int k=0;k<tries;k++){
			double c2[][][] = randomsubset(correspondence,samplesize) ;
			CameraModel model = new CameraModel(widthperdepth, widthperdepth*imageheight/imagewidth, 100, 100) ;
			model.calibrate(c2,imagewidth, imageheight) ;
			double error = model.error() ;
			if(error < besterror){
				besterror = error ;
				bestmodel = model ;
			}
		}
		return bestmodel ;
	}


	//returns a random sampling of calibration points
	public static double[][][] randomsubset(double c[][][], int amount){
		double c2[][][] = new double[amount][][] ;
		for(int k=0;k<amount;k++){
			c2[k] = c[(int)(Math.random()*c.length)] ;
		}
		return c2 ;
	}


}
