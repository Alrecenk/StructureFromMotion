/*This class is a simple template for openGL in java with texturing.
	It is not part of the structure from motion application(relevant methods have been copied into Main for that project)
	but I built it and find it useful for rapid prototyping of openGL apps in Java, and figured you might as well.

	It shows how to load and display textured quads as well as set-up for event driven methods for keyboard and mouse input.
*/
import java.nio.FloatBuffer;
import org.lwjgl.BufferUtils;
import org.lwjgl.input.Keyboard;
import org.lwjgl.input.Mouse;
import org.lwjgl.opengl.ARBTransposeMatrix;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.DisplayMode;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GLContext;

public class LWJGLTest implements EventDrivenInput{

	//window dimensions
	int windowwidth=800, windowheight=600;

	//catches mouse and keyboard input on the window in an event driven way
	lwjglinputcatcher input ;

	//image to be loaded and texture to be stored in
	String imagename = "derpy_hooves_is_licking_your_screen-by-umbra_neko.png" ;
	Texture pic ;

	// position of the box 
	float x = 0, y = 0, z = -5,size = 1;
	float rotation = 0;


	public void start() {

		createWindow(windowwidth,windowheight,"LWJGL Window") ;

		pic = new Texture(Utility.loadimage(imagename)) ;//load a texture from disk and into video memory

		while (!Display.isCloseRequested()) {
			

			update(1/60.0f);
			render();

			Display.update();
			Display.sync(60); // cap fps to 60fps
		}

		Display.destroy();
	}

	public boolean createWindow(int w, int h, String title) {
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


			if (!GLContext.getCapabilities().GL_ARB_transpose_matrix) {
				GL11.glLoadIdentity();
			} else {
				final FloatBuffer identityTranspose = BufferUtils
				.createFloatBuffer(16).put(
						new float[] { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
								0, 0, 0, 1 });
				identityTranspose.flip();
				ARBTransposeMatrix.glLoadTransposeMatrixARB(identityTranspose);
			}

			float widthperheight = windowwidth/(float)windowheight ;
			GL11.glFrustum(-widthperheight, widthperheight, -1, 1, 1f, 10000.0f);
			GL11.glMatrixMode(GL11.GL_MODELVIEW);
			GL11.glClearDepth(1.0);
			return true;
		} catch (Exception e) {
			return false;
		}
		
	}

	public static void drawTexturedQuad(float[][] points,  Texture texture) {

		GL11.glEnable( GL11.GL_TEXTURE_2D );
		GL11.glEnable(GL11.GL_BLEND) ;
		GL11.glBlendFunc(GL11.GL_SRC_ALPHA, GL11.GL_ONE_MINUS_SRC_ALPHA) ;

	
		GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_PRIORITY, 10) ;
		texture.bind();
		GL11.glBegin(GL11.GL_QUADS);


		GL11.glTexCoord2f(0.0f, 0.0f);
		GL11.glVertex3f(points[0][0], points[0][1], points[0][2]);

		GL11.glTexCoord2f(0.0f, texture.hRatio);
		GL11.glVertex3f(points[1][0], points[1][1], points[1][2]);

		GL11.glTexCoord2f(texture.wRatio, texture.hRatio);
		GL11.glVertex3f(points[2][0], points[2][1], points[2][2]);

		GL11.glTexCoord2f(texture.wRatio, 0.0f);
		GL11.glVertex3f(points[3][0], points[3][1], points[3][2]);


		GL11.glEnd();
	}

	public void update(float delta) {


		//move object around

		rotation += 150f * delta; // rotate 
		//input can still be read in a non-event driven way if desired
		if (Keyboard.isKeyDown(Keyboard.KEY_LEFT)) x -= 3.5f * delta;
		if (Keyboard.isKeyDown(Keyboard.KEY_RIGHT)) x += 3.5f * delta;

		if (Keyboard.isKeyDown(Keyboard.KEY_UP)) y += 3.5f * delta;
		if (Keyboard.isKeyDown(Keyboard.KEY_DOWN)) y -= 3.5f * delta;

		// keep object on the screen
		if (x < z) x = z;
		if (x > -z) x = -z;
		if (y < z) y = z;
		if (y > -z) y = -z;


	}





	public void render() {
		// Clear The Screen And The Depth Buffer
		GL11.glClear(GL11.GL_COLOR_BUFFER_BIT | GL11.GL_DEPTH_BUFFER_BIT);

		GL11.glPushMatrix();
		
		//rotate around self
		GL11.glTranslatef(x, y, z);
		GL11.glRotatef(rotation, .3f, 1f, .5f);
		GL11.glTranslatef(-x, -y, -z);

		// draw a box
		GL11.glColor3f(1,1,1) ;
		drawTexturedQuad(new float[][]{
				new float[]{x - size, y - size,z-size},
				new float[]{x + size, y - size,z-size},
				new float[]{x + size, y + size,z-size},
				new float[]{x - size, y + size,z-size}
		}, pic) ;
		drawTexturedQuad(new float[][]{
				new float[]{x - size, y - size,z+size},
				new float[]{x + size, y - size,z+size},
				new float[]{x + size, y + size,z+size},
				new float[]{x - size, y + size,z+size}
		}, pic) ;

		GL11.glColor3f(1,.3f,.3f) ;
		drawTexturedQuad(new float[][]{
				new float[]{x + size, y - size,z-size},
				new float[]{x + size, y - size,z+size},
				new float[]{x + size, y + size,z+size},
				new float[]{x + size, y + size,z-size}
		}, pic) ;
		GL11.glColor3f(.3f,.3f,1) ;
		drawTexturedQuad(new float[][]{
				new float[]{x - size, y - size,z-size},
				new float[]{x - size, y - size,z+size},
				new float[]{x - size, y + size,z+size},
				new float[]{x - size, y + size,z-size}
		}, pic) ;
		GL11.glColor3f(.3f,1,.3f) ;
		drawTexturedQuad(new float[][]{
				new float[]{x - size, y - size,z-size},
				new float[]{x + size, y - size,z-size},
				new float[]{x + size, y - size,z+size},
				new float[]{x - size, y - size,z+size}
		}, pic) ;
		GL11.glColor3f(1,.3f,1) ;
		drawTexturedQuad(new float[][]{
				new float[]{x - size, y + size,z-size},
				new float[]{x + size, y + size,z-size},
				new float[]{x + size, y + size,z+size},
				new float[]{x - size, y + size,z+size}
		}, pic) ;

		GL11.glPopMatrix();
	}

	public static void main(String[] argv) {
		LWJGLTest testapp = new LWJGLTest();
		testapp.start();
	}

	public void KeyEvent(int keycode, char keychar, boolean state) {
		// key events for this demo are handled in a non-event driven way as a demonstration 
		//see update(delta)

	}

	
	public void MouseEvent(int button, boolean state) {
		if(state){ // if clicking down
			float mx = Mouse.getX() ;
			float my = Mouse.getY() ;
			//move box x and y to line up with mouse
			x = -2*z*(mx - windowwidth/2.0f)/windowheight ;
			y = -2*z*(my - windowheight/2.0f)/windowheight ;
		}

		

	}
}
