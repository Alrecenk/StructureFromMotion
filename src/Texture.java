/* This class wraps an openGL texture in video memory 
 * and allows the creation of such a thing from a normal BufferedImage object in Java
 */

import java.nio.*;
import org.lwjgl.opengl.*;
import org.lwjgl.BufferUtils;
import java.awt.Image;
import java.awt.image.BufferedImage;
import java.awt.image.PixelGrabber;


public class Texture {
	public float wRatio;
	public float hRatio;
	public int tWidth;
	public int tHeight;

	private int textureID;
	public int width;
	public int height;

	public Texture(int textureID) { 
		this.textureID = textureID;	
	}
	
	//this constructor just wraps the static texture generation functions
	public Texture( BufferedImage bufferedImage){
		Texture t = generateTexture(bufferedImage) ; //it's easier to pass back a texture from the generation functions
		wRatio = t.wRatio ;
		hRatio = t.hRatio ;
		tWidth = t.tWidth ;
		tHeight = t.tHeight ;
		textureID = t.textureID ;
		width = t.width ;
		height = t.height ;
	}
	
	//destroy this texture from video memory
	public void destroy() {
		IntBuffer ibuf = BufferUtils.createIntBuffer(1);
		ibuf.put(textureID);
		ibuf.flip();
		GL11.glDeleteTextures(ibuf);
		
		ibuf.clear();
		
	}
	
	public void bind() { GL11.glBindTexture(GL11.GL_TEXTURE_2D, textureID);	}
	public int getID() { return textureID; }
	public void setWidth(int width) { this.width = width; }
	public void setHeight(int height) {	this.height = height; }
	
	public void setRatios() {
		wRatio = ((float) width) / tWidth;
		hRatio = ((float) height) / tHeight;
	}
	
	//create a texture from a BufferedImage
	public static Texture generateTexture(BufferedImage bufferedImage) {
		return generateTexture(generateTextureByteBuffer(bufferedImage),bufferedImage.getWidth(),bufferedImage.getHeight()) ;
	}
	
	//create a texture from a ByteBuffer of an image
	public static Texture generateTexture(ByteBuffer imageBuffer, int width, int height) {
		GL11.glEnable(GL11.GL_TEXTURE_2D);
		int theight, twidth;
		
		for(theight = 2; theight < height; theight *= 2) ;
		for(twidth = 2; twidth < width ; twidth *= 2) ;
		IntBuffer tmp = BufferUtils.createIntBuffer(1);
		GL11.glGenTextures(tmp);
		int textureID = tmp.get(0);
		Texture texture = new Texture(textureID);
		GL11.glBindTexture(GL11.GL_TEXTURE_2D, textureID);
		texture.setWidth(width);
		texture.setHeight(height);
		texture.tWidth = twidth;
		texture.tHeight = theight;
		
		GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MIN_FILTER, GL11.GL_LINEAR);
		
		GL11.glTexImage2D(GL11.GL_TEXTURE_2D, 0, GL11.GL_RGBA, twidth, theight, 0, GL11.GL_RGBA , GL11.GL_UNSIGNED_BYTE, imageBuffer);

		texture.setRatios();
		return texture;
	}
	
	//create a ByteBuffer of an image from the BufferedImage
	public static ByteBuffer generateTextureByteBuffer(BufferedImage bufferedImage) {
		int theight, twidth;
		int width = bufferedImage.getWidth(), height = bufferedImage.getHeight() ;
		
		for(theight = 2; theight < height; theight *= 2) ;
		for(twidth = 2; twidth < width ; twidth *= 2) ;
		
		int pixels[] = convertimagetopixels(bufferedImage);
		byte data[] = new byte[theight*twidth*4] ;
		
		int k,j,x,y ;
		for(x=0;x<width;x++){
		for(y=0;y<height;y++){
			while(x>=width)x-=width;
			while(y>=height)y-=height;
			k = x+y*width ;//image index
			j = 4* (x + y * twidth) ;//data index
			data[j] = ((byte)((pixels[k]>>16)&0xff)) ;//R
			data[j+1] = ((byte)((pixels[k]>>8)&0xff)) ;//G
			data[j+2] =((byte)((pixels[k]>>0)&0xff)) ;//B
			data[j+3] = ((byte)((pixels[k]>>24)&0xff)) ;//A
			
		}}
		ByteBuffer buffer = ByteBuffer.allocateDirect(data.length) ;
		buffer.put(data);
		buffer.flip();
		
		return buffer ;
		
	}
	
	//converts an image to an argb int array
	public static int[] convertimagetopixels(Image img) {
		int[] pixels = new int[img.getWidth(null) * img.getHeight(null)];
		PixelGrabber pg = new PixelGrabber(img, 0, 0, img.getWidth(null), img.getHeight(null), pixels, 0, img.getWidth(null));
		try {
		    pg.grabPixels();
		} catch (InterruptedException e) {
		    System.err.println("interrupted waiting for pixels!");
		}
		
		return pixels;
	} 
}