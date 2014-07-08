
/* Main class for automated 3D structure from motion
 * Arguments are widthperdepth, then optical flow opsteps, extrapolations, and sample resolution
 * 
 * This class makes use of OpticalFlow and CameraModel for the most complex operations, 
 * however, the actual 3D model construction, drawing, and saving to disk are handled directly in this file.
 */
import java.awt.* ;
import java.awt.event.* ;
import javax.swing.* ;
import org.lwjgl.BufferUtils;
import org.lwjgl.input.Keyboard;
import org.lwjgl.input.Mouse;
import org.lwjgl.opengl.ARBTransposeMatrix;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.DisplayMode;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GLContext;
import java.awt.image.BufferedImage ;
import java.awt.image.DataBufferInt ;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.nio.FloatBuffer;
import java.awt.image.BufferStrategy ;

public class Main extends JFrame
implements ActionListener,MouseListener, KeyListener, MouseMotionListener, Runnable, EventDrivenInput
{

	private static final long serialVersionUID = 1L;

	private Container pane ;
	BufferedImage display;
	DataBufferInt databuffer ;
	int displaypixels[];
	static int w=850,h=850;
	BufferStrategy strategy ;

	BufferedImage firstimage, secondimage ;
	String firstimagename ;
	int maxwidth=1280,maxheight = 768 ; //scales down to fit in 1024x768 or 1280x720

	BufferedImage firstdisplay, seconddisplay ;
	int displaywidth = 640, displayheight = 384 ; //scale down even more to display on the screen

	OpticalFlow flow ;
	CameraModel model ;
	float modelX[] = new float[3] ;
	float modelsize =1 ;

	// width of area covered by image relative to depth (i.e. field of view but as a ratio and not an angle)
	static double widthperdepth = 0; 

	//control parameters for optical flow
	static int firstkernelsize =32 ;
	static int opsteps = 0 ;
	static int extrapolations  =0;
	static int sampleresolution =0;
	static int maxlineiter = 30 ;

	//colors used for drawing point correspondences
	Color colors[] = new Color[]{Color.white, Color.black, Color.red, Color.blue, Color.green, Color.magenta, Color.yellow} ;


	Thread openglThread ;
	Texture texture ;
	int windowwidth,windowheight ;
	// position and rotation of the model relative to the camera
	float x = 0, y = 0, z = -10;
	float rotation = 0, rotation2 = 0;

	//catches mouse and keyboard input on the window in an event driven way
	lwjglinputcatcher input ;


	double modelsurface[][][] ; //A grid of 3D point making up the model's final surface
	boolean surfacevalid[][] ; //true or false whether a point on the surface is valid




	public static void main(String[] args)
	{
		if(args.length==4){
			widthperdepth = Double.parseDouble(args[0]) ;
			opsteps = Integer.parseInt(args[1]) ;
			extrapolations =  Integer.parseInt(args[2]);
			sampleresolution =  Integer.parseInt(args[3]);
		}else{
			System.out.println("Runtime parameters are width per depth, op steps, extrapolations, and sample resolution. Try 0.5 12 5 30 if you don't know what to use.");
			System.exit(0);
		}

		//boot up the 2D JFrame window
		Main window = new Main();
		window.init() ;
		window.addWindowListener(new WindowAdapter()
		{ public void windowClosing(WindowEvent e) { System.exit(0); }});

		window.setSize(w, h);
		window.setVisible(true);
	}


	public void init()
	{
		pane = getContentPane();
		pane.addMouseListener(this);
		pane.addMouseMotionListener(this);
		pane.addKeyListener(this);
		pane.requestFocus();
		Timer clock = new Timer(10, this); 
		clock.start();


		//ask for two images and then scalethem to the operating size
		firstimage = selectImage();
		if(firstimage == null)System.exit(0) ;
		firstimage = Utility.makethumbnail(firstimage, maxwidth, maxheight);
		secondimage = selectImage();
		if(secondimage == null)System.exit(0) ;
		secondimage = Utility.makethumbnail(secondimage, maxwidth, maxheight);

		firstdisplay = Utility.makethumbnail(firstimage, displaywidth, displayheight);
		seconddisplay = Utility.makethumbnail(secondimage, displaywidth, displayheight);

		System.out.println("Initializing Flow Structure...") ;
		flow = new OpticalFlow(firstimage,secondimage) ;
		System.out.println("Performing Optical Flow (may take a few minutes)...");
		long starttime = System.currentTimeMillis() ;
		flow.heriarchicalFlow(firstkernelsize, opsteps, extrapolations, sampleresolution, maxlineiter) ;
		System.out.println("Optical Flow Completed(" + (System.currentTimeMillis()-starttime)+"ms)!") ;
		
		
		System.out.println("Determining camera location and orientation (may take a few minutes)...");
		starttime = System.currentTimeMillis() ;
		double c[][][] = flow.getcorrespondences() ;
		model = CameraModel.RandomSampleCalibration(widthperdepth,firstimage.getWidth(), firstimage.getHeight(), c, 20, 8) ;
		System.out.println("Camera Calibration Completed(" + (System.currentTimeMillis()-starttime)+"ms)!") ;


		starttime = System.currentTimeMillis() ;
		buildModelSurface();
		reconstructInvalidSurface() ;
		System.out.println("Building Model Completed(" + (System.currentTimeMillis()-starttime)+"ms)!") ;

		//boot openGL window to display 3D model
		openglThread = new Thread(this) ;
		openglThread.start();
	}

	//Returns an image selected by the user or null on cancel
	public BufferedImage selectImage(){
		JFileChooser chooser = new JFileChooser("./");
		int returnVal = chooser.showOpenDialog(this);
		String filename="" ;
		if(returnVal == JFileChooser.APPROVE_OPTION){
			filename= chooser.getSelectedFile().getPath()  ;
			String s[] = filename.split("\\.");
			String ext = s[s.length-1].toLowerCase();
			if(ext.equals("jpg") || ext.equals("jpeg") || ext.equals("png") || ext.equals("gif") ){//load an image if this has an image extensiom
				if(firstimagename==null){
					firstimagename = chooser.getSelectedFile().getName() ;
				}
				return Utility.loadimage(filename) ;
			}
		}
		return null ;
	}

	public void paint(Graphics g)
	{
		if(display==null){
			//initialize image for double buffered display
			createBufferStrategy(2);
			strategy = getBufferStrategy();
			display = new BufferedImage(w,h,BufferedImage.TYPE_INT_RGB);
			databuffer = (DataBufferInt) display.getRaster().getDataBuffer();
			displaypixels = databuffer.getData();
		}

		Graphics g2 = strategy.getDrawGraphics();
		paint(displaypixels);
		g2.drawImage(display,0,0,this);
		paint2(g2);
		strategy.show();

	}

	//this is where pixel specific stuff gets drawn first
	public void paint(int displaypixels[]){
		//This method intentionally left blank. Application is built on a template with more features than needed. 
	}

	// java graphics stuff is drawn over pixel items
	public void paint2(Graphics g){
		int fx=20,fy=30,sy=15 ; //position(f) and spacing (s) of images on screen
		double scale = displaywidth/(double)maxwidth ;
		if(seconddisplay!=null){
			//draw images
			g.drawImage(firstdisplay,fx,fy,this) ;
			g.drawImage(seconddisplay,fx,fy+sy + firstdisplay.getHeight(),this) ;
			//draw optical flow correspondences over the images
			if(flow!=null){
				double matches[][][] = flow.getcorrespondences() ;
				for(int k=0;k<matches.length;k++){
					int x1 = fx + (int)(matches[k][0][0]*scale) ;
					int y1 = fy + (int)(matches[k][0][1]*scale) ;
					int x2 = fx + (int)(matches[k][1][0]*scale) ;
					int y2 = fy + sy + + firstdisplay.getHeight() + (int)(matches[k][1][1]*scale) ;
					g.setColor(colors[k%colors.length]) ;
					g.drawLine(x1-3,y1,x1+3,y1) ;
					g.drawLine(x1,y1-3,x1,y1+3) ;
					g.drawLine(x2-3,y2,x2+3,y2) ;
					g.drawLine(x2,y2-3,x2,y2+3) ;
				}
			}
		}
	}
	
	//used for the timer
	public void actionPerformed(ActionEvent e ){
		//This method intentionally left blank. Application is built on a template with more features than needed. 
		repaint();
	}

	//keyboard events for the 2D window
	public void keyPressed(KeyEvent e){
		//This method intentionally left blank. Application is built on a template with more features than needed. 
	}
	
	public void keyTyped(KeyEvent e){
		//This method intentionally left blank. Application is built on a template with more features than needed.
	}
	public void keyReleased(KeyEvent e)
	{
		//This method intentionally left blank. Application is built on a template with more features than needed.
	}




	//nouse events for the 2D window
	public void mousePressed(MouseEvent e)
	{
		pane.requestFocus();
	}
	public void mouseClicked(MouseEvent e)
	{
	}

	public void mouseReleased(MouseEvent e)
	{
	}

	public void mouseEntered(MouseEvent e)
	{
	}

	public void mouseExited(MouseEvent e)
	{
	}


	public void mouseMoved(MouseEvent e)
	{
	}

	public void mouseDragged(MouseEvent e)
	{
	}


	//This method run the secondary thread that boots and operates the openGL window
	public void run() {

		createopenGLWindow(w,h,"LWJGL Window") ;// boot up the openGL window
		texture = new Texture(firstimage) ;//load a texture from regular memory into video memory

		// save the model out to disk (couldn't do this until texture was created, which could only be done on openGL thread)
		exportSurface("Reconstruction") ; 

		//iterate at 60fps on the openGL window
		while (!Display.isCloseRequested()) {
			update(1/60.0f);
			render();
			Display.update();
			Display.sync(60); // cap fps to 60fps
		}

		Display.destroy();
	}

	//creates a basic openGL window with event driven input and the given width, height, and title
	public boolean createopenGLWindow(int w, int h, String title) {
		windowwidth = w ;
		windowheight = h ;
		try {
			Display.setDisplayMode(new DisplayMode(w, h));
			Display.setTitle(title);
			Display.create();

			if(input!=null){//close previously running inputthread if necessarry
				input.exiting = true ;
			}
			input = new lwjglinputcatcher(this,20);//open new input catcher

			GL11.glDisable(GL11.GL_CULL_FACE);
			GL11.glEnable(GL11.GL_DEPTH_TEST);
			GL11.glMatrixMode(GL11.GL_PROJECTION);

			//load identity matrix
			if (!GLContext.getCapabilities().GL_ARB_transpose_matrix) {
				GL11.glLoadIdentity();
			} else {
				final FloatBuffer identityTranspose = BufferUtils
						.createFloatBuffer(16).put(
								new float[] { 
										1, 0, 0, 0,
										0, 1, 0, 0,
										0, 0, 1, 0,
										0, 0, 0, 1 });
				identityTranspose.flip();
				ARBTransposeMatrix.glLoadTransposeMatrixARB(identityTranspose);
			}
			
			//create projection matrix via Fl frustum
			float widthperheight = windowwidth/(float)windowheight ;
			GL11.glFrustum(-widthperheight, widthperheight, -1, 1, 1f, 10000.0f);
			GL11.glMatrixMode(GL11.GL_MODELVIEW);
			GL11.glClearDepth(1.0);
			return true;
		} catch (Exception e) {
			return false;
		}

	}

	//allow control of rotating model with keyboard (called constantly at 60fps)
	public void update(float delta) {
		
		//input can still be read in a non-event driven way if desired
		
		//rotate with arrow keys
		if (Keyboard.isKeyDown(Keyboard.KEY_LEFT)) rotation -= 65f * delta;
		if (Keyboard.isKeyDown(Keyboard.KEY_RIGHT)) rotation += 65f * delta;
		if (Keyboard.isKeyDown(Keyboard.KEY_UP)) rotation2 -= 65f * delta;
		if (Keyboard.isKeyDown(Keyboard.KEY_DOWN)) rotation2 += 65f * delta;
		
		//zoom in and out with numpad keys
		if (Keyboard.isKeyDown(Keyboard.KEY_NUMPAD8)) z -= 5f * delta;
		if (Keyboard.isKeyDown(Keyboard.KEY_NUMPAD2)) z += 5f * delta;
	}


	//builds the model surface, assumes Optical Flow and Calibration are completed
	public void buildModelSurface(){
		double totalX[] = new double[3] ;
		double totalX2[] = new double[3] ;
		int xamount =0;

		modelsurface = new double[flow.grid1.length][flow.grid1[0].length][] ;
		surfacevalid = new boolean[flow.grid1.length][flow.grid1[0].length] ;

		for(int x=0;x<flow.grid1.length-1;x++){
			for(int y=0;y<flow.grid1.length-1;y++){
				//get image points from optial flow grid
				double m1[][] = new double[][]{
						new double[]{flow.grid1[x][y][0],flow.grid1[x][y][1]},
						new double[]{flow.grid2[x][y][0],flow.grid2[x][y][1]},
				};

				//use cameram odel to get 3D point
				double p[] = modelsurface[x][y] = model.get3Dpoint(m1, firstimage.getWidth(), firstimage.getHeight()) ;

				//determine if this point is a valid one
				if(!Utility.containsnan(p) && model.get3DError(m1,firstimage.getWidth(), firstimage.getHeight()) <0.1){
					surfacevalid[x][y]=true ;
					//get sums for mean and standard deviation calculations
					totalX[0]+=p[0];
					totalX[1]+=p[1];
					totalX[2]+=p[2];
					totalX2[0]+=p[0]*p[0];
					totalX2[1]+=p[1]*p[1];
					totalX2[2]+=p[2]*p[2];
					xamount++;
				}
			}
		}
		
		//calculate model location and size
		modelX[0] = (float)(totalX[0]/xamount );
		modelX[1] = (float)(totalX[1]/xamount );
		modelX[2] = (float)(totalX[2]/xamount );
		modelsize = (float)Math.sqrt((totalX2[0] - totalX[0]*totalX[0]/xamount)/xamount) ;


		//perform some additional validation that makes use of model location or size
		totalX = new double[3] ;
		totalX2 = new double[3] ;
		xamount =0;

		//if point is too far away from average of surrounding points then it is invalid
		boolean surfacevalid2[][] = new boolean[surfacevalid.length][surfacevalid[0].length] ;
		for(int x=1;x<flow.grid1.length-1;x++){
			for(int y=1;y<flow.grid1.length-1;y++){
				if(surfacevalid[x][y]){
					//has to have everything next to it be valid
					surfacevalid2[x][y] = surfacevalid[x][y] && surfacevalid[x+1][y] && surfacevalid[x-1][y] && surfacevalid[x][y+1] && surfacevalid[x][y-1] ;
					if(surfacevalid2[x][y]){
						//has to be close enough to the average of the surrounding points
						double avg[] = new double[]{ 
								0.25*(modelsurface[x-1][y][0]+modelsurface[x+1][y][0]+modelsurface[x][y-1][0]+modelsurface[x][y+1][0]),
								0.25*(modelsurface[x-1][y][1]+modelsurface[x+1][y][1]+modelsurface[x][y-1][1]+modelsurface[x][y+1][1]),
								0.25*(modelsurface[x-1][y][2]+modelsurface[x+1][y][2]+modelsurface[x][y-1][2]+modelsurface[x][y+1][2])
						} ;

						double dist = Math.sqrt((modelsurface[x][y][0]-avg[0])*(modelsurface[x][y][0]-avg[0])
								+(modelsurface[x][y][1]-avg[1])*(modelsurface[x][y][1]-avg[1])
								+(modelsurface[x][y][2]-avg[2])*(modelsurface[x][y][2]-avg[2])) ;
						surfacevalid2[x][y] = (dist < 10.0*modelsize/surfacevalid.length) ;
					}

				}
				//recompute model center and deviation with points removed
				if(surfacevalid2[x][y]){
					double p[] = modelsurface[x][y] ;
					totalX[0]+=p[0];
					totalX[1]+=p[1];
					totalX[2]+=p[2];
					totalX2[0]+=p[0]*p[0];
					totalX2[1]+=p[1]*p[1];
					totalX2[2]+=p[2]*p[2];
					xamount++;
				}
			}
		}

		surfacevalid = surfacevalid2 ;
		modelX[0] = (float)(totalX[0]/xamount );
		modelX[1] = (float)(totalX[1]/xamount );
		modelX[2] = (float)(totalX[2]/xamount );
		modelsize = (float)Math.sqrt((totalX2[0] - totalX[0]*totalX[0]/xamount)/xamount) ;




	}


	//interpolates or extrpolates from nearby points to attempt to recover invalid points
	public void reconstructInvalidSurface(){
		boolean surfacevalid2[][]  ;
		boolean anyinvalid = true ;
		//keep interpolating and extrapolating until there are no invalid points
		while(anyinvalid){
			anyinvalid = false ;
			surfacevalid2 = new boolean[surfacevalid.length][surfacevalid[0].length ] ;
			for(int x=1;x<surfacevalid.length-1;x++){
				for(int y=1;y<surfacevalid[x].length-1;y++){
					surfacevalid2[x][y] = true ;
					if(!surfacevalid[x][y]){
						//average points on opposite sides
						if(surfacevalid[x-1][y] && surfacevalid[x+1][y]){
							modelsurface[x][y][0] = 0.5*(modelsurface[x+1][y][0] + modelsurface[x-1][y][0]) ;
							modelsurface[x][y][1] = 0.5*(modelsurface[x+1][y][1] + modelsurface[x-1][y][1]) ;
							modelsurface[x][y][2] = 0.5*(modelsurface[x+1][y][2] + modelsurface[x-1][y][2]) ;
						}else if(surfacevalid[x][y-1] && surfacevalid[x][y+1]){
							modelsurface[x][y][0] = 0.5*(modelsurface[x][y+1][0] + modelsurface[x][y-1][0]) ;
							modelsurface[x][y][1] = 0.5*(modelsurface[x][y+1][1] + modelsurface[x][y-1][1]) ;
							modelsurface[x][y][2] = 0.5*(modelsurface[x][y+1][2] + modelsurface[x][y-1][2]) ;
						
						//extrapolate from 2 points on the same side
						}else if( x > 1 && surfacevalid[x-1][y] && surfacevalid[x-2][y]){
							modelsurface[x][y][0] = 2*modelsurface[x-1][y][0] - modelsurface[x-2][y][0] ;
							modelsurface[x][y][1] = 2*modelsurface[x-1][y][1] - modelsurface[x-2][y][1] ;
							modelsurface[x][y][2] = 2*modelsurface[x-1][y][2] - modelsurface[x-2][y][2] ;
						}else if( y > 1 && surfacevalid[x][y-1] && surfacevalid[x][y-2]){
							modelsurface[x][y][0] = 2*modelsurface[x][y-1][0] - modelsurface[x][y-2][0] ;
							modelsurface[x][y][1] = 2*modelsurface[x][y-1][1] - modelsurface[x][y-2][1] ;
							modelsurface[x][y][2] = 2*modelsurface[x][y-1][2] - modelsurface[x][y-2][2] ;
						}else if( x < surfacevalid.length-2 && surfacevalid[x+1][y] && surfacevalid[x+2][y]){
							modelsurface[x][y][0] = 2*modelsurface[x+1][y][0] - modelsurface[x+2][y][0] ;
							modelsurface[x][y][1] = 2*modelsurface[x+1][y][1] - modelsurface[x+2][y][1] ;
							modelsurface[x][y][2] = 2*modelsurface[x+1][y][2] - modelsurface[x+2][y][2] ;
						}else if( y < surfacevalid[0].length-2 && surfacevalid[x][y+1] && surfacevalid[x][y+2]){
							modelsurface[x][y][0] = 2*modelsurface[x][y+1][0] - modelsurface[x][y+2][0] ;
							modelsurface[x][y][1] = 2*modelsurface[x][y+1][1] - modelsurface[x][y+2][1] ;
							modelsurface[x][y][2] = 2*modelsurface[x][y+1][2] - modelsurface[x][y+2][2] ;
						
						}else{//if we couldn't fix it then set it to not valid
							surfacevalid2[x][y] = false ;
							anyinvalid = true ;
						}
					}
				}
			}
			surfacevalid = surfacevalid2 ;
		}


	}



	public void render() {
		// Clear The Screen And The Depth Buffer
		GL11.glClear(GL11.GL_COLOR_BUFFER_BIT | GL11.GL_DEPTH_BUFFER_BIT);

		GL11.glPushMatrix();

		//correct for model size and location as well as applying camera position and rotation
		GL11.glScalef(-3.0f/modelsize,-3.0f/modelsize,3.0f/modelsize);
		GL11.glTranslatef(x, y, z);
		GL11.glRotatef(rotation, 0f, 1f, 0f);
		GL11.glRotatef(rotation2, 1f, 0f, 0f);

		GL11.glTranslatef(-modelX[0], -modelX[1], -modelX[2]);


		//Configure and bind texture
		GL11.glColor3f(1,1,1) ;
		GL11.glEnable( GL11.GL_TEXTURE_2D );
		GL11.glEnable(GL11.GL_BLEND) ;
		GL11.glBlendFunc(GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA) ;
		GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_PRIORITY, 10) ;
		texture.bind();
		
		//draw the quads
		GL11.glBegin(GL11.GL_QUADS);
		for(int x=0;x<modelsurface.length-1;x++){
			for(int y=0;y<modelsurface.length-1;y++){
				//only draw quads if all points are valid
				if(surfacevalid[x][y] && surfacevalid[x+1][y] && surfacevalid[x][y+1] && surfacevalid[x+1][y+1]){
					GL11.glTexCoord2f((float)(flow.grid1[x][y][0]*texture.wRatio/texture.width), (float)(flow.grid1[x][y][1]*texture.hRatio/texture.height));
					GL11.glVertex3f((float)modelsurface[x][y][0], (float)modelsurface[x][y][1], (float)modelsurface[x][y][2]);
					
					GL11.glTexCoord2f((float)(flow.grid1[x+1][y][0]*texture.wRatio/texture.width), (float)(flow.grid1[x+1][y][1]*texture.hRatio/texture.height));
					GL11.glVertex3f((float)modelsurface[x+1][y][0], (float)modelsurface[x+1][y][1], (float)modelsurface[x+1][y][2]);
					
					GL11.glTexCoord2f((float)(flow.grid1[x+1][y+1][0]*texture.wRatio/texture.width), (float)(flow.grid1[x+1][y+1][1]*texture.hRatio/texture.height));
					GL11.glVertex3f((float)modelsurface[x+1][y+1][0], (float)modelsurface[x+1][y+1][1], (float)modelsurface[x+1][y+1][2]);
					
					GL11.glTexCoord2f((float)(flow.grid1[x][y+1][0]*texture.wRatio/texture.width), (float)(flow.grid1[x][y+1][1]*texture.hRatio/texture.height));
					GL11.glVertex3f((float)modelsurface[x][y+1][0], (float)modelsurface[x][y+1][1], (float)modelsurface[x][y+1][2]);
				}
			}
		}

		GL11.glEnd();//finish drawing quads
		GL11.glPopMatrix();//undo openGL camera operations
		
	}

	//openGL event driven keyboard response method
	public void KeyEvent(int keycode, char keychar, boolean state) {
		// key events for this demo are handled in a non-event driven way 
		//see update(delta)
	}
	
	//openGL event driven mouse response method
	public void MouseEvent(int button, boolean state) {
		
	}

	//saves the model created to disk as a wavefront OBJ with an associated MTL file
	public void exportSurface(String filename){
		File modfile = new File(filename +".obj") ;
		File matfile = new File(filename +".mtl") ;
		try{
			BufferedWriter writer = new BufferedWriter(new FileWriter(modfile));

			//link the material file
			writer.write("mtllib " + filename +".mtl \n\r") ;
			//write each vertex
			for(int y=0;y<modelsurface[0].length;y++){
				for(int x=0;x<modelsurface.length;x++){
					if(surfacevalid[x][y]){
						writer.write("v " + (float)modelsurface[x][y][0] + " " + -(float)modelsurface[x][y][1] + " " + (float)-modelsurface[x][y][2] +"\n\r");
						writer.write("vt " + (float)(flow.grid1[x][y][0]/texture.width) +" " + (float)((1-flow.grid1[x][y][1]/texture.height) ) + "\n\r");
						writer.write("vn 0 0 -1\n\r");
					}else{
						//gotta put something in for every vertex or the numbering for the quads gets screwed up
						writer.write("v 0 0 0\n\r");
						writer.write("vt 0 0\n\r");
						writer.write("vn 0 0 -1\n\r");
					}
				}
			}
			
			//make use of the material
			writer.write("usemtl material0\n\r") ;
			
			//write the quad indices (but only if valid)
			for(int y=0;y<modelsurface[0].length-1;y++){
				for(int x=0;x<modelsurface.length-1;x++){
					if(surfacevalid[x][y] && surfacevalid[x+1][y] && surfacevalid[x][y+1] && surfacevalid[x+1][y+1]){
						int i = 1 + x + y* modelsurface.length ;
						writer.write("f " 
								+ i +"/"+i+"/"+i +" " 
								+ (i+1) +"/"+(i+1)+"/"+(i+1) +" "
								+ (i+1+modelsurface.length) +"/"+(i+1+modelsurface.length)+"/"+(i+1+modelsurface.length) +" " 
								+ (i+modelsurface.length) +"/"+(i+modelsurface.length)+"/"+(i+modelsurface.length) +"\n\r" 
								);
					}

				}
			}

			writer.close();

			//write out the material file (which is fixed except for the texture name)
			writer = new BufferedWriter(new FileWriter(matfile));
			writer.write("newmtl material0\n");
			writer.write("Ka 1.000000 1.000000 1.000000\n\r");
			writer.write("Kd 1.000000 1.000000 1.000000\n\r");
			writer.write("Ks 0.000000 0.000000 0.000000\n\r");
			writer.write("Tr 1.000000\n");
			writer.write("illum 1\n\r");
			writer.write("Ns 0.000000\n\r");
			writer.write("map_Kd "+ firstimagename+"\n\r");
			writer.close();
		}catch(Exception e){
			System.err.print(e);
			e.printStackTrace();
		}

	}

}