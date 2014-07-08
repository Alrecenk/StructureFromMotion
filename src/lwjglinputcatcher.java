/*Creates a new thread that catches LWJGL window inputs and passes them to another object in an event driven way 
 * (implement EventDrivenInput to use this method)
 */

import org.lwjgl.input.Keyboard;
import org.lwjgl.input.Mouse;

public class lwjglinputcatcher{

	EventDrivenInput parent ;
	public boolean exiting =false;
	public int rawx,rawy;
	public int sleeptime ;
	public lwjglinputcatcher(EventDrivenInput p, int s ){
		sleeptime = s ;
		parent = p ;
		try{
			Mouse.create();
			Keyboard.create();
		}catch(Exception e){
			System.err.print(e) ;
		}
		Thread t = new Thread(new inputthread()) ;
		t.setPriority(Thread.MIN_PRIORITY) ;
		t.start();
		Keyboard.enableRepeatEvents(false) ;
	}

	private class inputthread implements Runnable{
		public void run(){
			while(!exiting){//loop until told to exit
				try{
					rawx = Mouse.getX() ;
					rawy = Mouse.getY() ;
					while(Mouse.next()){//process all mouse events available
						parent.MouseEvent(Mouse.getEventButton(),Mouse.getEventButtonState());//send mouse events to reciever
					}
					while(Keyboard.next()){//process all key events available
						parent.KeyEvent(Keyboard.getEventKey(), Keyboard.getEventCharacter(), Keyboard.getEventKeyState() ) ;
					}
					sleep();
				}catch(IllegalStateException e){
					exiting = true ;
					System.out.println("Window closed. Input Monitor Shutting Down.");
				}catch(Exception e){
					exiting = true ;
					System.err.print(e);
					e.printStackTrace() ;
				}
			}

		}

	}


	public void sleep(){
		try{Thread.sleep(sleeptime);}catch(Exception e){}
	}

}


