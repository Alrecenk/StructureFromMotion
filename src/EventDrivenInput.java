/* This interface can be implemented by an openGL main class to allow use of the LWJGL input catcher
*/

public interface EventDrivenInput{
	
	//called every time the mouse moves or a button is pressed or a button is released
	//if button = -1 this is a movement call
	//if button = 0 then left mouse click
	//fi button == 1 then right mouse
	//button numbers(-1=movement, 0=left button, 1 = right button, 2 = middle button
	public abstract void MouseEvent(int button,boolean state) ;
	
	//called every time a keyboard key is pressed or released
	public abstract void KeyEvent(int keycode, char keychar, boolean state) ;
}


