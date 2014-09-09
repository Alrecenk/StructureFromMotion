/* This class contains static utility methods for use with other applications.
 * Specifically it contains optimization methods, vector and matrix operations, and a few image processing functions.
 * 
 * Yay, anti-pattern created by Java's stubborn requirement that all functions be part of a class and the inability to add static methods to interfaces.
 */

import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.MediaTracker;
import java.awt.Toolkit;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.awt.image.PixelGrabber;

import javax.swing.JFrame;



public class Utility {

	
	//Returns the local minimum by using the LBFGS method starting from xp
	//maxiter is the maximum iterations of LBFGS to perform
	//stepiter is the maximum iterations to perform in line search
	//tolerance maximum gradient norm of error used an an exit condition
	//minimprovement is a fraction of the error removed in this step considered as an exit condition
	public static boolean verbose = false;
	public static double[] optimumbyLBFGS(OptimizationProblem problem, double[] xp, int m, int maxiter, int stepiter, double tolerance, double minimprovement)
	{

		int iter = 0;
		double error = problem.error(xp);
		double lasterror = Double.MAX_VALUE;
		double[] gradient = problem.gradient(xp);
		//Console.Out.WriteLine("Start error: " + error + "  gradient norm :" + norm(gradient)) ;
		//error = feval(fun,x.p,1);
		//gradient = feval(fun,x.p,2);
		double[] g = gradient;
		//preallocate arrays
		int k = 0, j;
		double[][] s = new double[m][];
		double[] rho = new double[m];
		double[][] y = new double[m][];
		double[] nw = new double[m];
		double[] r, q;
		double B;

		//quit when acceptable accuracy reached or iterations exceeded
		while (iter < maxiter && norm(gradient) > tolerance && lasterror-error > minimprovement*error){
			lasterror = error;
			iter = iter + 1;
			k = k + 1;
			g = gradient;
			//%initial pass just use gradient descent
			if (k == 1 || xp.length ==1){
				r = g;
			}else{
				//two loop formula
				q = g;
				int i = k - 1;
				while (i >= k - m && i > 0){
					j = i % m; //%index into array operating as a fixed size queue
					nw[j] = rho[j] * dot(s[j], q);
					q = subtract(q, scale(y[j], nw[j]));
					i = i - 1;
				}
				j = (k - 1) % m;
				r = scale(q, dot(s[j], y[j]) / dot(y[j], y[j]));// % gamma,k * q = H0k q
				i = Math.max(k - m, 1);
				while (i <= k - 1){
					j = i % m; //%index into array operating as a fixed size queue
					B = rho[j] * dot(y[j], r);

					r = add(r, scale(s[j], nw[j] - B));
					i = i + 1;
				}
			}
			//% get step size
			// alfa = StepSize(fun, x, -r, 1,struct('c1',10^-4,'c2',.9,'maxit',100)) ;
			double alfa = stepSize(problem, xp, scale(r, -1), 1, stepiter, .1, .9);

			//%apply step and update arrays
			j = k % m;
			s[j] = scale(r, -alfa);
			if (containsnan(s[j]) || alfa <= 0){
				System.err.println("Invalid exit condition in LBFGS!" + alfa);
				return xp;
			}
			xp = add(xp, s[j]);
			gradient = problem.gradient(xp);
			error = problem.error(xp);
			y[j] = subtract(gradient, g);
			rho[j] = 1.0 / dot(y[j], s[j]);
		}
		return xp;
	}
	
	
	//starting from w0 searches for a weight vector using Newton's method
	//and Wolfe condition line-search until the gradient magnitude is below tolerance
	//or a maximum number of iterations is reached.
	public double[] newtonMethod(OptimizationProblem problem, double w0[], double tolerance, int maxiter){
		double w[] = w0 ;
		double gradient[] = problem.gradient(w0) ;
		int iteration = 0 ;
		while(Math.sqrt(dot(gradient,gradient)) > tolerance && iteration < maxiter){
			iteration++;
			//get the second derivative matrix
			double hessian[][] = problem.hessian(w) ;

			//perform an LDL decomposition and substitution to solve the system of equations Hd = -g  for the Newton step
			//(See Linear Least Squares Article on Inductive Bias for details on this technique)

			//calculate the LDL decomposition in place with D over top of the diagonal
			for (int j = 0; j < w.length; j++){
				for (int k = 0; k < j; k++){//D starts as Hjj then subtracts
					hessian[j][j] -= hessian[j][k] * hessian[j][k] * hessian[k][k];//Ljk^2 * Dk
				}
				for (int i = j + 1; i < w.length; i++){//L over the lower diagonal
					for (int k = 0; k < j; k++){//Lij starts as Hij then subtracts
						hessian[i][j] -= hessian[i][k] * hessian[j][k] * hessian[k][k];//Ljk^2*D[k]
					}
					hessian[i][j] /= hessian[j][j];//divide Lij by Dj
				}
			}

			//check if D elements are all positive to make sure Hessian was positive definite and Newton step goes to a minimum
			boolean positivedefinite = true ;
			for(int k=0;k<w.length&&positivedefinite;k++){
				positivedefinite &= hessian[k][k] > 0 ;
			}

			//right hand side for Newton's method is negative gradient
			double[] newton = scale(gradient,-1);
			if(positivedefinite){ // if H was pd then get newton direction, otherwise leave it as -gradient
				//in-place forward substitution with L
				for (int j = 0; j < w.length; j++){
					for (int i = 0; i < j; i++){
						newton[j] -= hessian[j][i] * newton[i];
					}
				}
				//Multiply by inverse of D matrix
				for (int k = 0; k < w.length; k++){//inverse of diagonal
					newton[k] /= hessian[k][k];//is 1 / each element
				}
				// in-place backward substitution with L^T
				for (int j = w.length - 1; j >= 0; j--){
					for (int i = j + 1; i < w.length; i++){
						newton[j] -= hessian[i][j] * newton[i];
					}
				}
			}

			//calculate step-size		
			double alpha = stepSize(problem, w, newton, 1, 500, 0.001, 0.9) ;// then use it
			//apply step
			w = add( w, scale( newton, alpha)) ;
			//calculate gradient for exit condition and next step 
			gradient = problem.gradient(w) ;
		}

		return w ;
	}


	//starting from w0 searches for a weight vector using gradient descent
	//and Wolfe condition line-search until the gradient magnitude is below tolerance
	//or a maximum number of iterations is reached.
	public static double[] gradientDescent(OptimizationProblem problem, double w0[], double tolerance, int maxiter){
		double w[] = w0 ;
		double gradient[] = problem.gradient(w0) ;
		int iteration = 0 ;
		while(Math.sqrt(dot(gradient,gradient)) > tolerance && iteration < maxiter){
			iteration++ ;
			//calculate step-size in direction of negative gradient
			double alpha = stepSize(problem, w, scale(gradient,-1), 1, 500, 0.1, 0.9) ;
			w = add( w, scale( gradient, -alpha)) ; // apply step
			gradient = problem.gradient(w) ; // get new gradient
		}
		return w ;
	}



	//Performs a binary search to satisfy the Wolfe conditions
	//returns alpha where next x =should be x0 + alpha*d 
	//guarantees convergence as long as search direction is bounded away from being orthogonal with gradient
	//x0 is starting point, d is search direction, alpha is starting step size, maxit is max iterations
	//c1 and c2 are the constants of the Wolfe conditions (0.1 and 0.9 can work)
	public static double stepSize(OptimizationProblem problem, double[] x0, double[] d, double alpha, int maxit, double c1, double c2){

		//get error and gradient at starting point  
		double fx0 = problem.error(x0);
		double gx0 = dot(problem.gradient(x0), d);

		//bound the solution
		double alphaL = 0;
		double alphaR = 1000;

		for (int iter = 1; iter <= maxit; iter++){
			double[] xp = add(x0, scale(d, alpha)); // get the point at this alpha
			double erroralpha = problem.error(xp); //get the error at that point
			if (erroralpha >= fx0 + alpha * c1 * gx0)	{ // if error is not sufficiently reduced
				alphaR = alpha;//move halfway between current alpha and lower alpha
				alpha = (alphaL + alphaR)/2.0;
			}else{//if error is sufficiently decreased 
				double slopealpha = dot(problem.gradient(xp), d); // then get slope along search direction
				if (slopealpha <= c2 * Math.abs(gx0)){ // if slope sufficiently closer to 0
					return alpha;//then this is an acceptable point
				}else if ( slopealpha >= c2 * gx0) { // if slope is too steep and positive then go to the left
					alphaR = alpha;//move halfway between current alpha and lower alpha
					alpha = (alphaL+ alphaR)/2;
				}else{//if slope is too steep and negative then go to the right of this alpha
					alphaL = alpha;//move halfway between current alpha and upper alpha
					alpha = (alphaL+ alphaR)/2;
				}
			}
		}

		//if ran out of iterations then return the best thing we got
		return alpha;
	}
	
	
	
	//loads an image from a file into BufferedImage
	//yes, it is loading into a regular image and then drawing to a buffered image
	//yes, that is stupid, but there doesn't seem to exist a better way to do it since ImageIO.read doesn't work reliably
	public static BufferedImage loadimage(String imagename){
		try{
			Toolkit toolkit = Toolkit.getDefaultToolkit();
			Image image = toolkit.getImage(imagename);
			MediaTracker mediaTracker = new MediaTracker(new JFrame());
			mediaTracker.addImage(image, 0);
			mediaTracker.waitForID(0);

			BufferedImage buf = new BufferedImage(image.getWidth(null), image.getHeight(null), BufferedImage.TYPE_INT_RGB);
			Graphics2D bufImageGraphics = buf.createGraphics();
			bufImageGraphics.drawImage(image, 0, 0, null);

			return buf ;
		}catch(Exception e){
			System.out.println("Image Load Failed: " + e ) ;
			e.printStackTrace() ;
		}

		return null ;
	}
	
	
	
	//returns a one dimensional gaussian kernel
	public static double[] gaussian(int radius){
		double g[] = new double[radius*2+1] ;
		double theta = radius/3.0 ;
		double oneovertwothetasquared = 1.0/(2*theta*theta);
		double total = 0 ;
		for(int x=-radius;x<=radius;x++){
			g[x+radius] = Math.exp(-oneovertwothetasquared*x*x) ;
			total+=g[x+radius] ;
		}
		total = 1.0/total ;
		for(int x=-radius;x<=radius;x++){
			g[x+radius]*=total ;
		}
		return g ;
	}
	
	
	//Applies a convolution kernel in both dimensions, mainly used for blurring images
	public static float[][][] applykernel(float[][][] i, double kernel[]){
		float[][][] i2 = new float[i.length][i[0].length][i[0][0].length];
		int radius = (kernel.length - 1 )/2 ;
		for(int x=radius;x<i.length-radius;x++){
			for(int y=radius;y<i[0].length-radius;y++){
				for(int c = 0 ; c < i[0][0].length; c++){
					double v = 0;
					for(int k=-radius;k<=radius;k++){
						//for(int j=-radius;j<=radius;j++){
							
								v+=i[x+k][y][c]*kernel[k+radius];
						//}
					}
					i2[x][y][c] = (float)v ;
				}
			}
		
		}
		
		float[][][] i3 = new float[i.length][i[0].length][i[0][0].length];
		for(int x=radius;x<i.length-radius;x++){
			for(int y=radius;y<i[0].length-radius;y++){
				for(int c = 0 ; c < i[0][0].length; c++){
					double v = 0;
					//for(int k=-radius;k<=radius;k++){
						for(int j=-radius;j<=radius;j++){
							
								v+=i2[x][y+j][c]*kernel[j+radius];
						}
					//}
					i3[x][y][c] = (float)v ;
				}
			}
		
		}
		
		return i3 ;
	}
	
	
	
	
	//given a symmetric positive definite matrix this function returns the LDL decomposition of that matrix
	//with the D part over the diagonal of the L
	//works great for linear least squares problems (i.e. XTX = X^T * X )
	public static double[][] LDL(double[][] XTX)
	{
		int inputs = XTX.length;
		//perform LDL Cholesky decomposition
		double[][] LDL = new double[inputs][ inputs];
		//double D[] = new double[inputs] ;
		for (int j = 0; j < inputs; j++)
		{
			LDL[j][j] = XTX[j][j];
			for (int k = 0; k < j; k++){
				LDL[j][j] -= LDL[j][k] * LDL[j][k] * LDL[k][k];
			}
			for (int i = j + 1; i < inputs; i++){
				LDL[i][j] = XTX[i][j];
				for (int k = 0; k < j; k++){
					LDL[i][j] -= LDL[i][k] * LDL[j][k] * LDL[k][k];
				}
				LDL[i][j] /= LDL[j][j];
			}
		}
		return LDL;
	}


	//Solves a System from an LDL decomposition using forward/backward substitution
	// L*D*L*(result) = (XTY)
	public static double[] solvesystem(double LDL[][], double XTY[]){
		//back substitution with L
		int inputs = LDL.length ;
		double p[] = new double[XTY.length] ;
		for (int j = 0; j < inputs; j++){
			p[j] = XTY[j] ;
			for (int i = 0; i < j; i++){
				p[j] -= LDL[j][i] * p[i];
			}
		}
		//Multiply by inverse of D matrix
		double q[] = new double[p.length] ;
		for (int k = 0; k < inputs; k++){//inverse of diagonal
			q[k] = p[k]/LDL[k][k];//is 1 / each element
		}
		// forward substitution with L^T
		double c[] = new double[q.length];
		for (int j = inputs - 1; j >= 0; j--){
			c[j] = q[j] ;
			for (int i = j + 1; i < inputs; i++){
				c[j] -= LDL[i][j] * c[i];
			}
		}
		return c ;
	}
	
	
	//returns the euclidean length of a vector
	public static double length(double[] a)
	{
		return Math.sqrt(dot(a, a));
	}

	//the two norm is just the squareroot of the dot product, same as length
	public static double norm(double[] a)
	{
		return length(a);
	}
	//dot product
	public static double dot(double[] a, double[] b)
	{
		double c = 0;
		for (int k = 0; k < a.length; k++)
		{
			c += a[k] * b[k];
		}
		return c;
	}
	//returns a vector = to a*s
	public static double[] scale(double[] a, double s)
	{
		double[] b = new double[a.length];
		for (int k = 0; k < a.length; k++)
		{
			b[k] = a[k] * s;
		}
		return b;
	}
	
	//returns a vector scaled to the given length
	public static double[] scaleTo(double[] a, double s){
		s/=length(a) ;
		double[] b = new double[a.length];
		for (int k = 0; k < a.length; k++)
		{
			b[k] = a[k] * s;
		}
		return b;
	}
	
	//returns the sum of two vectors
	public static double[] add(double[] a, double[] b)
	{
		double[] c = new double[a.length];
		for (int k = 0; k < a.length; k++)
		{
			c[k] = a[k] + b[k];
		}
		return c;
	}
	//returns the difference of two vectors = a-b
	public static double[] subtract(double[] a, double[] b)
	{
		double[] c = new double[a.length];
		for (int k = 0; k < a.length; k++)
		{
			c[k] = a[k] - b[k];
		}
		return c;
	}

	//squared distance between vectors
	public static double distancesquared(double[] a, double[] b){
		double dst = 0 ;
		for(int k=0; k<a.length; k++){
			dst += (a[k]-b[k])*(a[k]-b[k]) ;
		}
		return dst ;
	}
	
	//the cross product (only works for 3D vectors)
	public static double[] cross(double[] a, double[] b){
		return new double[]{ a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1] - a[1]*b[0]} ;
	}
	
	//multiplies a vector by a matrix
	public static double[] multiply(double mat[][], double vec[]){
		double newvec[] = new double[mat.length];
		for(int i=0;i<mat.length;i++){
			for(int j=0;j<vec.length;j++){
				newvec[i]+=vec[j]*mat[i][j] ;
			}
		}
		return newvec ;

	}
	
	
	//returns true ifthere are any "not a number" or infinity values in the given vector
	public static boolean containsnan(double[] x){
		for (int k = 0; k < x.length; k++){
			if (Double.isNaN(x[k]) || Double.isInfinite(x[k])){
				return true;
			}
		}
		return false;
	}
	
	
	
	//scales an image, maintaining aspect ratio, to fit within the desired width and height
	//averages color over squares to get good results when scaling down
	//use scaleto when scaling up
	public static BufferedImage makethumbnail(BufferedImage img, double desiredwidth, double desiredheight){
		//System.out.println("Original Image size: " + img.getWidth(null)+ " x "+img.getHeight(null));
		if(img ==null || img.getWidth(null) <1 || img.getHeight(null) <1){
			return null;
		}else{
			byte image[][][] = convertimage(img) ;
			byte[][][] newimage = makethumbnail(image,desiredwidth,desiredheight) ;
			BufferedImage img2 = convertimage(newimage) ;
			return img2 ;
		}

	}


	//scales an image, maintaining aspect ratio, to fit within the desired width and height
	//averages color over squares to get good results when scaling down
	//use scaleto when scaling up
	public static byte[][][] makethumbnail(byte[][][] image, double desiredwidth, double desiredheight){

		int width = image.length ;
		int height = image[0].length ;
		System.out.println(width +", " + height + " --> " + desiredwidth +", " + desiredheight) ;
		double scale = Math.min(desiredwidth/(double)width, desiredheight/(double)height) ;
		//System.out.println(desiredwidth + ", " + desiredheight + " --> " + scale);
		int nw = (int)(width*scale+.1), nh = (int)(height*scale+.1) ;
		System.out.println(nw +", " + nh ) ;

		byte newimage[][][] = new byte[nw][nh][3];


		double pixel[] = new double[4];
		double xstep = width/(double)nw, ystep=height/(double)nh;
		for(int x = 0 ; x < nw;x++){
			for(int y = 0 ; y < nh;y++){
				pixel[0] =  x*xstep ;
				pixel[1] = y*ystep ;
				pixel[2] = pixel[0]+xstep;
				pixel[3] = pixel[1]+ystep;
				double c[] = colorsampled(pixel, image);
				newimage[x][y][0] = (byte)(c[0]+.5);
				newimage[x][y][1] = (byte)(c[1]+.5);
				newimage[x][y][2] = (byte)(c[2]+.5);
			}
		}


		return newimage ;


	}


	//returns the average color of an axis aligned bounding box on this texture
	//using no interpolation 
	//AABB is an axis aligned bounding box of the form{ minx,miny,maxx,maxy}
	public static double[] colorsampled(double AABB[], byte image[][][]){


		int width = image.length;
		int height = image[1].length ;

		int minx = (int)(AABB[0]) ;
		int maxx = (int)(AABB[2]+1) ;
		int miny = (int)(AABB[1]) ;
		int maxy = (int)(AABB[3]+1) ;
		//make sure AABB doesn't try to read outside of texture
		if(minx<0)minx = 0 ;
		if(miny<0)miny = 0;
		if(maxx>width)maxx = width ;
		if(maxy>height)maxy = height ;

		//area*each color
		double rarea = 0 ;
		double barea = 0 ;
		double garea = 0 ;
		double area =0,a;
		for(int x=minx;x<maxx;x++){
			for(int y=miny;y<maxy;y++){
				double[] texel = new double[]{x,y,x+1,y+1};
				double intersect[] = AABBintersect(texel,AABB) ;
				if(AABB[2] > AABB[0] && AABB[3] > AABB[1]){//if the AABBs intersect
					double intersectarea = (AABB[2]-AABB[0])*(AABB[3]-AABB[1]);
					if(intersectarea>0.000001){
						area += intersectarea ;
						rarea+= (image[x][y][0]&0xff)*intersectarea ;
						garea+= (image[x][y][1]&0xff)*intersectarea ;
						barea+= (image[x][y][2]&0xff)*intersectarea ;
					}
				}
			}
		}
		return new double[]{rarea/area,garea/area,barea/area};
	}
	
	//returns an AABB representing the intersection of two AABBs 
	//where AABBs given as xmin,ymin,xmax,ymax
	public static double[] AABBintersect(double[] a, double[] b){
		return new double[]{Math.max(a[0], b[0]),Math.max(a[1], b[1]), Math.min(a[2], b[2]),Math.min(a[3], b[3])};
	}
	
	//converts a bufferedimage image to 3 integer arrays
	//first index is x, then y, then channel
	public static byte[][][] convertimage(BufferedImage img) {
		int w= img.getWidth(null), h = img.getHeight(null) ;
		int[] pixels = new int[ w*h ];
		PixelGrabber pg = new PixelGrabber(img, 0, 0, w, h, pixels, 0, w);
		try {
			pg.grabPixels();
		} catch (InterruptedException e) {
			System.err.println("interrupted waiting for pixels!");
		}
		byte data[][][] = new byte[w][h][3];
		for(int x=0;x<w;x++){
			for(int y=0;y<h;y++){
				int k = x+y*w;
				data[x][y][0] =  (byte)((pixels[k]>>16)&0xff) ;
				data[x][y][1] =  (byte)((pixels[k]>>8)&0xff) ;
				data[x][y][2] =  (byte)((pixels[k])&0xff) ;
			}
		}

		return data;
	}
	
	//converts a set of 3 byte arrays back into a bufferedimage
	public static BufferedImage convertimage(byte i[][][]){
		int w = i.length,h=i[0].length ;
		int p[] = new int[w*h] ;
		for(int x=0;x<w;x++){
			for(int y=0;y<h;y++){
				p[x+y*w] = (i[x][y][0]&0xff)<<16 | (i[x][y][1]&0xff) << 8 | (i[x][y][2]&0xff) ;

			}
		}
		return convertpixelstoimage(p,w);
	}
	
	//takes an ARGB int array and converts to a buffered image
	public static BufferedImage convertpixelstoimage(int p[], int w){
		BufferedImage image = new BufferedImage(w, p.length/w, BufferedImage.TYPE_INT_RGB);
		DataBufferInt databuffer = (DataBufferInt) image.getRaster().getDataBuffer();
		int[] ipix = databuffer.getData();
		for(int k=0;k<p.length;k++){
			ipix[k] = p[k] ;
		}
		return image ;
	}
	
	
	
	
}
