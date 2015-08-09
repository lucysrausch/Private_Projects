
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.net.InetSocketAddress;



import org.eclipse.jetty.server.Handler;
import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.SessionManager;
import org.eclipse.jetty.server.handler.ContextHandler;
import org.eclipse.jetty.server.handler.DefaultHandler;
import org.eclipse.jetty.server.handler.HandlerList;
import org.eclipse.jetty.server.handler.ResourceHandler;
import org.eclipse.jetty.servlet.ServletContextHandler;
import org.eclipse.jetty.util.log.Log;
import org.eclipse.jetty.util.log.Logger;
import org.eclipse.jetty.util.thread.QueuedThreadPool;

public class Launcher {
	public static void main(String[] args) throws Exception {

		Server s = new Server(new InetSocketAddress("0.0.0.0",
				8081));
		
		ServletContextHandler h = new ServletContextHandler(
				ServletContextHandler.SESSIONS);
		
		
		
		h.setInitParameter(SessionManager.__SessionCookieProperty,
				"Node-Session");
		h.addServlet(MeinTestServer.class, "/*");
		HandlerList hl = new HandlerList();
		hl.setHandlers(new Handler[] { h });
		s.setHandler(hl);
		s.start();
	}
	
}