/* This class uses a combination coordinate descent and approximate Newton's method 
 * to calculate optical flow between two images taken close together.
 * A quadrilateral grid is laid over each image, then the images are blurred significantly
 * each point on the quadrilateral is adjusted by building a parabola approximation to matching error around that point
 * and then line-searching in the direction of that parabola's minimum (quasi-newton optimization).
 * This process is repeated a few times for every point in the grid. 
 * Then the grid is subdivided (quadrupling the number of quads), and more optimization steps are done
 * Then the blur is reduced, and more optimization steps are done
 * This process is repeated several times, ultimately resulting in 2 bilinear splines that describe a mapping from one image to the other.
 * 
 * This technique was designed to be used on frames of video for the purpose of video super resolution.
 * It works best when the images are taken close together with the same camera under the same conditions
 * and have a small number of large continuous surfaces as the method assumes a continuous surface.
 * Like most optical flow methods it performs poorly on regions of solid color.
 */
  
import java.awt.image.BufferedImage;
import java.awt.image.PixelGrabber;
import java.util.ArrayList;

public class OpticalFlow {

	//the raw image data given to this optical flow
	int rawwidth, rawheight ;
	public BufferedImage ri1, ri2 ;
	public float rawimage1[][][];//indices are x,y,{r,g,b}
	public float rawimage2[][][];//indices are x,y,{r,g,b}

	//the smoothed and/or scaled image data currently being used for processing
	int width, height;
	int gaussiansize ;//size of blur kernel ( 3 * gamma)
	public float image1[][][];//indices are x,y,{r,g,b}
	public float image2[][][];//indices are x,y,{r,g,b}

	//the current model of quads on each image that are being matched
	float grid1[][][] ; // indices are: x index in the grid, y index in the grid, then {x,y} of the point
	float grid2[][][] ; // indices are: x index in the grid, y index in the grid, then {x,y} of the point


	//data about current execution step
	int currentgridindex[] ; // current {x,y} of the point on the grid being optimized
	int samplesteps = 20 ; // resolution of samples taken for calculating matching error
	int maxlinesearchiterations = 20 ; // maximum iterations used in step size search
	double gradientepsilon = 1 ; // epsilon used for numerically calculating directional gradient in step-size search



	//relative point locations of samples to use for building approximating parabolas
	//must contain at least 5 points, more points may increase accuracy of model
	double modelscale = 1 ;
	double model[][] = new double[][]{
			new double[]{0,modelscale},
			new double[]{0,-modelscale},
			new double[]{modelscale,0},
			new double[]{-modelscale,0},
			new double[]{modelscale,modelscale},
			new double[]{modelscale,-modelscale},
			new double[]{-modelscale,-modelscale},
			new double[]{-modelscale,modelscale}
	} ;

	//matrices derived from model used to quickly calculate quadratic approximations
	double X[][] = new double[model.length][] ;
	double[][] XTX ;//XTX is symmetric (only lower triangle defined)
	double ModelLDL[][] ; // LDL decomposition with D over diagonal, doesn't change as long as model points are constant



	//initialize an optical flow object given 2 raw images
	public OpticalFlow(BufferedImage ri1, BufferedImage ri2){
		this.ri1 = ri1;
		this.ri2 = ri2 ;
		//moved images into float format
		rawwidth = ri1.getWidth();
		rawheight = ri1.getHeight();
		rawimage1 = convertimage(ri1) ;
		rawimage2 = convertimage(ri2) ;
		//start grid as single quads moved in from edges
		int w = rawwidth, h = rawheight ;
		grid1 = new float[][][]{
				new float[][]{ new float[]{w/8f,h/8f}, new float[]{w/8f,h*7/8f}},
				new float[][]{ new float[]{w*7/8f,h/8f}, new float[]{w*7/8f,h*7/8f}}
		} ;

		grid2 = new float[][][]{
				new float[][]{ new float[]{w/8f,h/8f}, new float[]{w/8f,h*7/8f}},
				new float[][]{ new float[]{w*7/8f,h/8f}, new float[]{w*7/8f,h*7/8f}}
		} ;

		currentgridindex = new int[]{0,0} ;
	}


	//runs a hierarchical optical flow routine beginning from a newly initialized OpticalFlow object
	//if this does not provide enough control you can call setfunctionimage, extrapolategrid, and gridoptimizationstep individually as desired
	//first kernel size is the size of the blur kernel for the outermost step (it will be halved as quads are subdivided) (32 works for 720p images)
	//opsteps is the number of optimization steps ran between each kernel change or grid subdivision (5 to 20 usually works)
	//extrapolations if the number of time the grid will be subdivided (try 3 to 6 , increasing it increases model size and run-time exponentially)
	//sample steps is the resolution at which quad matching error is measured (try 20)
	//max line search iterations is the max iterations spent on step-size search (20 is reasonable)
	public void heriarchicalFlow(int firstkernelsize, int opsteps, int extrapolations, int samplesteps, int maxlinesearchiterations){

		int kernelsize = firstkernelsize;
		this.samplesteps = samplesteps ;
		this.maxlinesearchiterations = maxlinesearchiterations ;

		setfunctionimage(kernelsize) ;
		for(int k =0 ; k < extrapolations;k++){

			//make the grid higher resolution
			extrapolategrid() ;

			//run a few grid optimization steps
			for(int j=0;j<opsteps;j++){
				gridoptimizationstep() ;
			}
			//make the blur kernel smaller (bring the image more into focus)
			kernelsize*=.5 ;

			//update the image with the new blur setting
			if(kernelsize < 1){
				setfunctionimagetoraw() ;
			}else{
				setfunctionimage(kernelsize) ;
			}

			//run a few more grid optimization steps
			for(int j=0;j<opsteps;j++){
				gridoptimizationstep() ;
			}
		}
	}


	//returns the optical flow in the format of a set of correlation points
	// first index is which point, secon is which image, then {x,y} in image coordinates
	public double[][][] getcorrespondences(){
		ArrayList<double[][]> list = new ArrayList<double[][]>() ;
		//add each grid point to the list
		for(int x= 0 ; x < grid1.length;x++){
			for(int y= 0 ;y < grid1[x].length;y++){
				//if all values in correlating point are numbers and finite
				if(!Double.isNaN(grid1[x][y][0]) && !Double.isInfinite(grid1[x][y][0])  &&
						!Double.isNaN(grid1[x][y][1]) && !Double.isInfinite(grid1[x][y][1])  &&
						!Double.isNaN(grid2[x][y][0]) && !Double.isInfinite(grid2[x][y][0])  &&
						!Double.isNaN(grid2[x][y][1]) && !Double.isInfinite(grid2[x][y][1])  
						){
					list.add(new double[][]{
							new double[]{grid1[x][y][0], grid1[x][y][1]},
							new double[]{grid2[x][y][0], grid2[x][y][1]}
					}) ;
				}
			}
		}
		double[][][] array= new double[list.size()][][] ;
		list.toArray(array) ;
		return array ;

	}

	//sets image1,image2 to smoothed version of the raw images
	public void setfunctionimage(int gaussianradius){
		width = rawwidth;
		height = rawheight;
		gaussiansize = gaussianradius ;
		double g[] = Utility.gaussian(gaussiansize) ;

		image1 = Utility.applykernel(rawimage1,g);
		image2 = Utility.applykernel(rawimage2,g);
	}



	//sets image1, and image2 to the raw image data
	public void setfunctionimagetoraw(){
		width = rawwidth;
		height = rawheight;
		image1 = rawimage1 ;
		image2 = rawimage2 ;

	}



	//turns each quad in the grid into 4 quads
	public void extrapolategrid(){
		float ng1[][][] = new float[grid1.length*2-1][grid1[0].length * 2 - 1 ][2] ;
		float ng2[][][] = new float[grid1.length*2-1][grid1[0].length * 2 - 1 ][2] ;
		for(int x= 0 ; x < grid1.length;x++){
			for(int y= 0 ;y < grid1[x].length;y++){
				ng1[x*2][y*2] = grid1[x][y] ;
				ng2[x*2][y*2] = grid2[x][y] ;
				if(x < grid1.length-1){
					ng1[x*2+1][y*2][0] = (grid1[x][y][0]+ grid1[x+1][y][0])/2f ;
					ng1[x*2+1][y*2][1] = (grid1[x][y][1]+ grid1[x+1][y][1])/2f ;
					ng2[x*2+1][y*2][0] = (grid2[x][y][0]+ grid2[x+1][y][0])/2f ;
					ng2[x*2+1][y*2][1] = (grid2[x][y][1]+ grid2[x+1][y][1])/2f ;

				}
				if(y < grid1[0].length-1){
					ng1[x*2][y*2+1][0] = (grid1[x][y][0]+ grid1[x][y+1][0])/2f ;
					ng1[x*2][y*2+1][1] = (grid1[x][y][1]+ grid1[x][y+1][1])/2f ;
					ng2[x*2][y*2+1][0] = (grid2[x][y][0]+ grid2[x][y+1][0])/2f ;
					ng2[x*2][y*2+1][1] = (grid2[x][y][1]+ grid2[x][y+1][1])/2f ;

				}
				if(x < grid1.length-1 && y < grid1[0].length-1){
					ng1[x*2+1][y*2+1][0] = (grid1[x][y][0]+ grid1[x][y+1][0]+ grid1[x+1][y][0] + grid1[x+1][y+1][0])/4f ;
					ng1[x*2+1][y*2+1][1] = (grid1[x][y][1]+ grid1[x][y+1][1]+ grid1[x+1][y][1] + grid1[x+1][y+1][1])/4f ;
					ng2[x*2+1][y*2+1][0] = (grid2[x][y][0]+ grid2[x][y+1][0]+ grid2[x+1][y][0] + grid2[x+1][y+1][0])/4f ;
					ng2[x*2+1][y*2+1][1] = (grid2[x][y][1]+ grid2[x][y+1][1]+ grid2[x+1][y][1] + grid2[x+1][y+1][1])/4f ;

				}
			}
		}
		grid1 = ng1 ;
		grid2 = ng2 ;

	}

	//optimizes every point on the grid once
	public void gridoptimizationstep(){
		for(int x= 0 ; x < grid1.length;x++){
			for(int y= 0 ;y < grid1[x].length;y++){
				pointoptimizationstep(x,y) ;
			}
		}

	}


	//performs one Newton Line-search step on the given grid-point
	public void pointoptimizationstep(int gridx, int gridy){
		currentgridindex[0] = gridx;
		currentgridindex[1] = gridy ;
		double xp[] = new double[]{grid2[gridx][gridy][0],grid2[gridx][gridy][1]} ;
		double basefunctionvalue = error(xp) ;//model is built relative to current function value
		//System.out.println("bfv:"+basefunctionvalue) ;

		//Model X values never change so we only need to calculate the decomposition for building the quadratic approximation once 
		if(ModelLDL ==null){
			//model is of the form	[x y][a c][x] + [d f][x] with input and output relative to the point the model is built around
			//						     [c b][y]        [y]


			for(int k=0;k<model.length;k++){
				X[k] = new double[]{model[k][0]*model[k][0],model[k][1]*model[k][1], 2*model[k][0]*model[k][1], model[k][0], model[k][1]} ;

			}

			XTX = new double[X[0].length][] ;//symmetric
			for (int i = 0; i < X[0].length; i++){//so we only have to
				XTX[i] = new double[i+1] ;//build the lower triangle
				for (int j = 0; j <= i; j++){
					for (int k = 0; k < model.length; k++){
						XTX[i][j] += X[k][i] * X[k][j];
					}
				}
			}
			ModelLDL = Utility.LDL(XTX) ; //get LDL decomposition with Utility function
			

		}
		double Y[] = new double[model.length] ;
		//get relative function values
		for(int k=0;k<model.length;k++){
			Y[k] = error(new double[]{model[k][0]+xp[0],model[k][1]+xp[1]}) - basefunctionvalue ;
		}
		//multiply Y's by X^T to get right hand side of least squares problem
		double XTY[] = new double[ModelLDL.length] ;
		for (int j = 0; j < X[0].length; j++){
			for (int k = 0; k < X.length; k++){
				XTY[j] += Y[k]* X[k][j];
			}
		}
		//solve the least squares problem to fit the quadratic approximation
		double quadratic[] = Utility.solvesystem(ModelLDL,XTY) ;

		//get the parameters of the parabola with friendlier names
		double a = quadratic[0] ;
		double b = quadratic[1] ;
		double c = quadratic[2] ;
		double d = quadratic[3] ;
		double f = quadratic[4] ;


		//if not positive definite then calculate what multiple would need to be added to make PSD and add 5% more than that to make PD
		if(a <= 0 || a*b - c*c <= 0){
			double i = 1.05*Math.max(-a, (-(a+b) + Math.sqrt( (a+b)*(a+b) - 4 * (a*b-c*c)))/2) ;
			a+=i ;
			b+=i ;
		}

		//solve [2a 2c]P = [d] to get search direction
		//	[2c 2b]    [f]
		double det = 2*( a*b - c*c) ;
		if(Math.abs(det) > 0.000001){
			double p[] = {-(b*d-c*f)/det, -(f*a-c*d)/det} ;//cramer's rule for solving 2x2 system
			double np[] =new double[2];
			//get step size with line search in that direction
			double s = stepSize(xp,p,1,maxlinesearchiterations,.1,.9) ;
			//apply step to grid point
			np[0] = xp[0] + s * p[0] ;
			np[1] = xp[1] + s * p[1] ;
			//System.out.println(s +" * ( " + p[0] +" , " + p[1] +" )") ;
			//only accept if better than what you had before
			if(error(np) < basefunctionvalue){
				grid2[gridx][gridy][0] = (float)(np[0]) ;
				grid2[gridx][gridy][1] = (float)(np[1]) ;
			}


		}



	}


	//returns the current error between images around the grid point at the given index
	//using the current grid1,grid2, image1, and image2 data
	public double pointerror(int gx,int gy, int quadsteps){
		double error  = 0 ;
		if(gx>0 && gy>0){
			error+=quaderror(gx-1,gy-1,quadsteps) ;
		}
		if(gx>0 && gy<grid1[0].length-1){
			error+=quaderror(gx-1,gy,quadsteps) ;
		}
		if(gx<grid1.length-1 && gy>0){
			error+=quaderror(gx,gy-1,quadsteps) ;
		}
		if(gx<grid1.length-1 && gy<grid1[0].length-1){
			error+=quaderror(gx,gy,quadsteps) ;
		}
		return error ;
	}




	//returns the sum of the squared error between the images under the quad from (gx1,gx2), to (gx1+1,gx2+1)
	public double quaderror(int gx, int gy, int steps){
		double error = 0 ;
		double edgemult;//edge points should only be weighted half as much with corners 1/4th
		for(int k=0;k<steps;k++){
			double t = k/(double)(steps-1) ;
			//weight edge points lower
			if(k == 0 || k == steps-1){
				edgemult = .5 ;
			}else{
				edgemult = 1 ;
			}
			for(int j=0;j<steps;j++){

				double s = j/(double)(steps-1) ;

				//linearly interpolate grid quad to get image points
				double p1[] =new double[]{
						grid1[gx][gy][0] * (1-t)*(1-s) + grid1[gx+1][gy][0] * t*(1-s) + grid1[gx][gy+1][0] * (1-t)*s + grid1[gx+1][gy+1][0] * t*s,
						grid1[gx][gy][1] * (1-t)*(1-s) + grid1[gx+1][gy][1] * t*(1-s) + grid1[gx][gy+1][1] * (1-t)*s + grid1[gx+1][gy+1][1] * t*s } ;

				double p2[] =new double[]{
						grid2[gx][gy][0] * (1-t)*(1-s) + grid2[gx+1][gy][0] * t*(1-s) + grid2[gx][gy+1][0] * (1-t)*s + grid2[gx+1][gy+1][0] * t*s,
						grid2[gx][gy][1] * (1-t)*(1-s) + grid2[gx+1][gy][1] * t*(1-s) + grid2[gx][gy+1][1] * (1-t)*s + grid2[gx+1][gy+1][1] * t*s } ;

				//any point outside of the image returns a large error
				if(p1[0] < 1 || p2[0] < 1 || p1[1] < 1 || p2[1]<1 || p1[0] > rawwidth-2 ||  p2[0] > rawwidth-2 ||  p1[1] > rawheight-2 ||  p2[1] > rawheight-2 ){
					return 200000000 ;
				}
				//get interpolated color of image at those points
				float c1[] = color(image1,p1) ;
				float c2[] = color(image2,p2) ;
				//add squared distanced corrected for edge and corner points
				if(j == 0 || j == steps-1){
					error+= .5 * edgemult *( (c1[0]-c2[0])*(c1[0]-c2[0]) + (c1[1]-c2[1])*(c1[1]-c2[1]) + (c1[2]-c2[2])*(c1[2]-c2[2]) ) ;
				}else{
					error+= edgemult * ( (c1[0]-c2[0])*(c1[0]-c2[0]) + (c1[1]-c2[1])*(c1[1]-c2[1]) + (c1[2]-c2[2])*(c1[2]-c2[2]) ) ;
				}
			}
		}
		
		return error ;
	}



	//returns the bilinearly interpolated color of the given point in the given image
	//input points are assumed to be in the space of the raw images, but this function returns the value from the processed/scaled images
	public float[] color(float image[][][], double rp[]){
		double p[] = new double[]{(rp[0] * width)/rawwidth, (rp[1] * height)/rawheight} ;
		int x = (int)p[0] ;
		int y = (int)p[1] ;
		double t = p[0] - x ;
		double s = p[1] - y ;
		double a = (1-t)*(1-s), b = t*(1-s), c = (1-t)*s, d = t*s ;
		return new float[]{
				(float)(image[x][y][0] * a + image[x+1][y][0] * b + image[x][y+1][0] * c + image[x+1][y+1][0] * d),
				(float)(image[x][y][1] * a + image[x+1][y][1] * b + image[x][y+1][1] * c + image[x+1][y+1][1] * d),
				(float)(image[x][y][2] * a + image[x+1][y][2] * b + image[x][y+1][2] * c + image[x+1][y+1][2] * d)
		} ;
	}


	//the function value used for optimization
	//x is typically just the location of the point being altered and so all other information needed
	//(such as what point it is and the amount of points to sample)
	//should be stored and accessible from here
	public double error(double x[]){
		
		float prevvalue[] = grid2[currentgridindex[0]][currentgridindex[1]] ;//save current location of point
		grid2[currentgridindex[0]][currentgridindex[1]] = new float[]{ (float)x[0], (float)x[1]} ;//put location you want to test in
		double error = pointerror(currentgridindex[0], currentgridindex[1], samplesteps) ;//get the error for that value
		grid2[currentgridindex[0]][currentgridindex[1]] = prevvalue ;//put the old value back
		
		return error ; //return error
	}


	//returns the gradient of the function at x dotted with d
	//used with line-search which never requires the full gradient
	public double dotGradient(double x[], double d[]){
		double p1[] = Utility.subtract(x, Utility.scaleTo(d,gradientepsilon)) ;
		double p2[] = Utility.add(x, Utility.scaleTo(d,gradientepsilon)) ;
		double f1 = error(p1) ;
		double f2 = error(p2) ;

		return (f2-f1)/(gradientepsilon*2) ;
	}


	//Performs a binary search to satisfy the Wolfe conditions
	//returns alpha where next x =should be x0 + alpha*d 
	//guarantees convergence as long as search direction is bounded away from being orthogonal with gradient
	//x0 is starting point, d is search direction, alpha is starting step size, maxit is max iterations
	//c1 and c2 are the constants of the Wolfe conditions (0.1 and 0.9 can work)
	//uses an approximate directional gradient since a true gradient is not defined for the optical flow error function
	public double stepSize(double[] x0, double[] d, double alpha, int maxit, double c1, double c2){

		//get error and gradient at starting point  
		double fx0 = error(x0);
		double gx0 = dotGradient(x0, d);

		//bound the solution
		double alphaL = 0;
		double alphaR = 100;

		for (int iter = 1; iter <= maxit; iter++){
			double[] xp = Utility.add(x0, Utility.scale(d, alpha)); // get the point at this alpha
			double erroralpha = error(xp); //get the error at that point
			if (erroralpha >= fx0 + alpha * c1 * gx0)	{ // if error is not sufficiently reduced
				alphaR = alpha;//move halfway between current alpha and lower alpha
				alpha = (alphaL + alphaR)/2.0;
			}else{//if error is sufficiently decreased 
				double slopealpha = dotGradient(xp, d); // then get slope along search direction
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




	//converts a BufferedImage to a float array format
	public static float[][][] convertimage(BufferedImage img) {
		int w= img.getWidth(null), h = img.getHeight(null) ;
		int[] pixels = new int[ w*h ];
		PixelGrabber pg = new PixelGrabber(img, 0, 0, w, h, pixels, 0, w);
		try {
			pg.grabPixels();
		} catch (InterruptedException e) {
			System.err.println("interrupted waiting for pixels!");
		}
		float data[][][] = new float[w][h][3];
		for(int x=0;x<w;x++){
			for(int y=0;y<h;y++){
				int k = x+y*w;
				data[x][y][0] =  (pixels[k]>>16)&0xff ;
				data[x][y][1] =  (pixels[k]>>8)&0xff ;
				data[x][y][2] =  (pixels[k])&0xff ;
			}
		}

		return data;
	}







}
