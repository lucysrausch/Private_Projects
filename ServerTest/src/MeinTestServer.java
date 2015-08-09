import java.io.IOException;
import java.io.PrintWriter;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.sql.Timestamp;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import javax.servlet.ServletException;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

import org.cacert.gigi.output.template.Outputable;
import org.cacert.gigi.output.template.Template;
import org.json.JSONArray;
import org.json.JSONObject;
import org.jsoup.Jsoup;
import org.jsoup.nodes.Document;
import org.jsoup.nodes.Element;
import org.jsoup.select.Elements;

public class MeinTestServer extends HttpServlet {
	
	
	int linkCounter = 0;
	String level = "test";

	String lastPage;
	ArrayList<String> tempArray = new ArrayList<String>();
	String route = "";
	String firstPage;
	String secondPage;
	String thirdPage;
	List<String> outputStringArray = new ArrayList<String>();
	String lastOutput = null;
	int hitlercount = 0;
	String lastURL1 = "yay", lastURL2 = "yay", lastURL3 = null;
	Elements links1 = null, links2 = null, links3 = null;
	
	String count = "count";
	String value = "moin";
	
	final String JDBC_DRIVER="com.mysql.jdbc.Driver";  
    final String DB_URL="jdbc:mysql://localhost/test";

    //  Database credentials
    final String USER = "root";
    final String PASS = "password";
    
    List<Integer> dataList = new ArrayList<Integer>();
    List<Date> timeList = new ArrayList<Date>();
    
    Connection conn = null;
	Statement stmt = null;
	
	private Template t;
	private DateFormat dateFormat;
	private Date date;

	public void init() throws ServletException {
		t = new Template(MeinTestServer.class.getResource("Wiki.templ"));
		dateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
		   //get current date time with Date()
		   date = new Date();
		super.init();
	}

	protected void doGet(HttpServletRequest req, HttpServletResponse resp)
			throws ServletException, IOException {
		handleRequest(req, resp, false);
	}

	protected void doPost(HttpServletRequest req, HttpServletResponse resp)
			throws ServletException, IOException {
		handleRequest(req, resp, true);
	}

	private void handleRequest(final HttpServletRequest req,
			final HttpServletResponse resp, final boolean post)
			throws IOException, ServletException {
		
		if (post == true) {
			
			
			
			/*
			   //get current date time with Calendar()
			   Calendar cal = Calendar.getInstance();
			   System.out.println(dateFormat.format(cal.getTime()));
			
			value = req.getParameter("value");
            count = req.getParameter("count");
			System.out.println("Value: " + value);
			System.out.println("Count: " + count);
			
			try {
		         // Register JDBC driver
		         Class.forName("com.mysql.jdbc.Driver");

		         // Open a connection
		         conn = DriverManager.getConnection(DB_URL,USER,PASS);

		         // Execute SQL query
		         stmt = conn.createStatement();
		         String sql;
		              
		         sql = "INSERT INTO Datapoints VALUES (" + count + ", " + value + ", '" + dateFormat.format(cal.getTime()) + "');";
		         System.out.println(sql);
		         stmt.executeUpdate(sql);
		         
		         stmt.close();
		         conn.close();     
			}catch(Exception e){
		         //Handle errors for Class.forName
		         e.printStackTrace();
		    }
		    */
		}
		
		//System.out.println(req);

			

		final String pathInfo = req.getPathInfo();
		// resp.getWriter().println(pathInfo);
		if (pathInfo.equals("/wikisearch")) {
		
			String url = null;
			lastOutput = null;
			boolean valid = false;
			url = req.getParameter("url");
			int count = 0;
			try {
			count = Integer.parseInt(req.getParameter("count")); }
			catch (Exception e) {}
			System.out.println(url);
			System.out.println(count);
		    hitlercount = 0;
		    if (url != null)
		    {
			if (url.contains("https://de.wikipedia.org/wiki/") || url.contains("https://en.wikipedia.org/wiki/") && count <= 5 && count > 0) {
				System.out.println("valid");
				valid = true;
		    try {
			firstPage = url.replace("https://de.wikipedia.org/wiki/", "");

		    	//firstPage = url.replace("https://en.wikipedia.org/wiki/", "");
			}
				catch (Exception e) {}
		    //System.out.println(searchPage("https://de.wikipedia.org/wiki/Vielv%C3%B6lkerstaat", 30));
		    
		    int i = 30000;
		    int c = 0;
		    String urlOrigin = url;
		    String newResult = null;
		    String urlResult = searchPage(urlOrigin, i, 1, false);
		    i = linkCounter;
		    System.out.println(i);
		    for (int a = 0; a < i; a++) {
		    	if (hitlercount >= count)
	    			break;
		    	urlResult = searchPage(urlOrigin, a, 2, true);
		    	searchPage(urlResult, 30000, 2, false);
		    	try {
		    		secondPage = urlResult.replace("https://de.wikipedia.org/wiki/", "");
		    		}
					catch (Exception e) {}
		    	c = linkCounter;
		    	for (int b = 0; b < c; b++) {
		    		if (hitlercount >= count)
		    			break;
		    		newResult = searchPage(urlResult, b, 3, true);
		    		try {
		    		    thirdPage = newResult.replace("https://de.wikipedia.org/wiki/", "");
		    		    }
						catch (Exception e) {}
		    				
		    		searchPage(newResult, 30000, 3, false);	
		    	}
		    }
			}
			else {
				outputStringArray.add("No valid URL (https://de.wikipedia.org/wiki/ or https://en.wikipedia.org/wiki/) or count > 5");
			}
		    }
		    
		    else {
		    	outputStringArray.add("rawr");
		    }
			

			


			Outputable o = new Outputable() {

				public void output(PrintWriter out, Map vars) {
					t.output(out, vars);
				}
			};
			
			HashMap<String, Object> vars = new HashMap<String, Object>();
			
			vars.put("message", "Hello user! " + req);

			vars.put("result1", outputStringArray.get(0));
			
			if (valid) {
				for (int i = 2; i < count + 1; i++) {
			vars.put("result" + i, outputStringArray.get(i-1));

				}
			}

			outputStringArray.clear();
			o.output(resp.getWriter(), vars);
			
			
			
            
		
		}
		
		/*
		if (pathInfo.equals("/test")) {
			try{
		         // Register JDBC driver
		         Class.forName("com.mysql.jdbc.Driver");

		         // Open a connection
		         conn = DriverManager.getConnection(DB_URL,USER,PASS);

		         // Execute SQL query
		         stmt = conn.createStatement();
		         String sql;
		              
		         sql = "SELECT id, count, timedate FROM Datapoints";
		         
		         ResultSet rs = stmt.executeQuery(sql);
		         
		         //float[] dat = new float[20];
		        

		         // Extract data from result set
		         while(rs.next()){
		            //Retrieve by column name
		            int id  = rs.getInt("id");
		            int value = rs.getInt("count");
		            Date SQLtime = rs.getTimestamp("timedate");
		            
		            dataList.add(value);
		            timeList.add(SQLtime);
		            
		            System.out.print("Date: " + dateFormat.format(SQLtime) + "    ");
		            System.out.print("ID: " + id + "    ");
		            System.out.println(", Value: " + value + " ");
		         }

		         // Clean-up environment
		         rs.close();
		         stmt.close();
		         conn.close();
		      }catch(SQLException se){
		         //Handle errors for JDBC
		         se.printStackTrace();
		      }catch(Exception e){
		         //Handle errors for Class.forName
		         e.printStackTrace();
		      }finally{
		         //finally block used to close resources
		         try{
		            if(stmt!=null)
		               stmt.close();
		         }catch(SQLException se2){
		         }// nothing we can do
		         try{
		            if(conn!=null)
		            conn.close();
		         }catch(SQLException se){
		            se.printStackTrace();
		         }//end finally try
		      } //end try

			Outputable o = new Outputable() {

				public void output(PrintWriter out, Map vars) {
					t.output(out, vars);
				}
			};
			
		    
		    JSONArray jsonArray = new JSONArray();
		    
			for (int i=0; i < dataList.size(); i++) {
				JSONArray jsonArrayRaw = new JSONArray();
				jsonArrayRaw.put(timeList.get(i).getTime());
				System.out.println(timeList.get(i).getTime());
				jsonArrayRaw.put(dataList.get(i));
				jsonArray.put(jsonArrayRaw);
			}
			
			dataList.clear();
			timeList.clear();
			
			HashMap<String, Object> vars = new HashMap<String, Object>();
			HashMap<String, Object> data = new HashMap<String, Object>();
			
			
			vars.put("message", "Hallo Benutzer! " + req);
			
			data.put("name", "Testreihe");
			
			data.put("data", jsonArray);
			
			vars.put("seriesdata", new JSONArray(
					new JSONObject[] { new JSONObject(data) }));
			o.output(resp.getWriter(), vars);
		}*/
	}
	
    private String searchPage(String url, int count, int level, boolean position) throws IOException {
        //print("Fetching %s...", url);
    	
    	try{
    		String tryURL = null;
    		
    	if (url.contains("https://de.wikipedia.org/wiki/") == false && url.contains("https://en.wikipedia.org/wiki/") == false)
    		return null;
    	if (url.contains(".ogg") || url.contains("#") || url.contains("&") || url.contains("%")) {
    		return null;
    	}
        boolean found = false;
        
        if (level == 1) {
        	if (lastURL1.equals(url) == false) {
        		Document doc1 = Jsoup.connect(url).get();
        		links1 = doc1.select("a[href]");    
        	}
        	lastURL1 = url;
        linkCounter = 0;
        String lookURL = null;
        //print("\nLinks: (%d)", links.size());
        
        if (position) {
        	return links1.get(count).attr("abs:href");
        }
        
        else {
        //System.out.println(url);
        for (Element link : links1) {
            lookURL = link.attr("abs:href");
            
            if (count == linkCounter) {
            	return lookURL;
            }
            
            linkCounter++;     
            	
            if (lookURL.equals("https://de.wikipedia.org/wiki/Adolf_Hitler") || lookURL.equals("https://en.wikipedia.org/wiki/Adolf_Hitler")) {
            	String output = null;
            	if (level == 1)
            		output = "Hitler detected!: " + firstPage + " ---> Hitler";
            	
            	else if (level == 2)
            		output = "Hitler detected!: " + firstPage + " ---> " + secondPage + " ---> Hitler";
            	
            	else if (level == 3)
            		output = "Hitler detected!: " + firstPage + " ---> " + secondPage + " ---> " + thirdPage + " ---> Hitler";

            	System.out.println(output);
            	System.out.println(hitlercount);

            	if (output.equals(lastOutput) == false && firstPage.equals(secondPage) == false && firstPage.equals(thirdPage) == false && secondPage.equals(thirdPage) == false && output.contains("#") == false) {
            	   outputStringArray.add(output);
            	   hitlercount++;
            	}
            	lastOutput = output;
            	
            	
            }
            
        }
        }
        }
        
        if (level == 2) {
        	if (lastURL2.equals(url) == false) {
        		Document doc2 = Jsoup.connect(url).get();
        		links2 = doc2.select("a[href]");    
        	}
        	lastURL2 = url;
            
            
            linkCounter = 0;
            String lookURL = null;
            //print("\nLinks: (%d)", links.size());
            
            if (position) {
            	return links2.get(count).attr("abs:href");
            }
            
            else {
            //System.out.println(url);
            for (Element link : links2) {
                lookURL = link.attr("abs:href");
                
                if (count == linkCounter) {
                	return lookURL;
                }
                
                linkCounter++;     
                	
                if (lookURL.equals("https://de.wikipedia.org/wiki/Adolf_Hitler") || lookURL.equals("https://en.wikipedia.org/wiki/Adolf_Hitler")) {
                	String output = null;
                	if (level == 1)
                		output = "Hitler detected!: " + firstPage + " ---> Hitler";
                	
                	else if (level == 2)
                		output = "Hitler detected!: " + firstPage + " ---> " + secondPage + " ---> Hitler";
                	
                	else if (level == 3)
                		output = "Hitler detected!: " + firstPage + " ---> " + secondPage + " ---> " + thirdPage + " ---> Hitler";

                	System.out.println(output);
                	System.out.println(hitlercount);

                	if (output.equals(lastOutput) == false && firstPage.equals(secondPage) == false && firstPage.equals(thirdPage) == false && secondPage.equals(thirdPage) == false && output.contains("#") == false) {
                	   outputStringArray.add(output);
                	   hitlercount++;
                	}
                	lastOutput = output;
                	
                	
                }
                
            }
            }
            }
        
        if (level == 3) {
        	//if (lastURL3.equals(url) == false) {
        		Document doc3 = Jsoup.connect(url).get();
        		links3 = doc3.select("a[href]");    
        	//}
        	lastURL3 = url;
            
            
            linkCounter = 0;
            String lookURL = null;
            //print("\nLinks: (%d)", links.size());
            
            if (position) {
            	return links3.get(count).attr("abs:href");
            }
            
            else {
            //System.out.println(url);
            for (Element link : links3) {
                lookURL = link.attr("abs:href");
                
                if (count == linkCounter) {
                	return lookURL;
                }
                
                linkCounter++;     
                	
                if (lookURL.equals("https://de.wikipedia.org/wiki/Adolf_Hitler") || lookURL.equals("https://en.wikipedia.org/wiki/Adolf_Hitler")) {
                	String output = null;
                	if (level == 1)
                		output = "Hitler detected!: " + firstPage + " ---> Hitler";
                	
                	else if (level == 2)
                		output = "Hitler detected!: " + firstPage + " ---> " + secondPage + " ---> Hitler";
                	
                	else if (level == 3)
                		output = "Hitler detected!: " + firstPage + " ---> " + secondPage + " ---> " + thirdPage + " ---> Hitler";

                	System.out.println(output);
                	System.out.println(hitlercount);

                	if (output.equals(lastOutput) == false && firstPage.equals(secondPage) == false && firstPage.equals(thirdPage) == false && secondPage.equals(thirdPage) == false && output.contains("#") == false) {
                	   outputStringArray.add(output);
                	   hitlercount++;
                	}
                	lastOutput = output;
                	
                	
                }
                
            }
            }
            }
    	}
    	
    	catch (Exception e)
    	{}
    	return null;
    }
}
