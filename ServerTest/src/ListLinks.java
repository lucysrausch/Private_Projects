import org.jsoup.Jsoup;
import org.jsoup.nodes.Document;
import org.jsoup.nodes.Element;
import org.jsoup.select.Elements;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;


/**
 * Example program to list links from a URL.
 */
public class ListLinks {
    static int linkCounter = 0;
	static String level = "test";

	String lastPage;
	static ArrayList<String> tempArray = new ArrayList<String>();
	static String route = "";
	static String firstPage;
	static String secondPage;
	static String thirdPage;
    public static void main(String[] args) throws IOException {
    	
    	System.out.println("Please enter Wiki URL:");
    	InputStreamReader isr = new InputStreamReader(System.in); 
	    BufferedReader br = new BufferedReader(isr); 
	    String url = br.readLine(); 
	    System.out.println("Searching: " + url); 
	    firstPage = url.replace("https://de.wikipedia.org/wiki/", "");
	    //System.out.println(searchPage("https://de.wikipedia.org/wiki/Vielv%C3%B6lkerstaat", 30));
	    
	    int i = 30000;
	    int c = 0;
	    String urlOrigin = url;
	    String newResult = null;
	    String urlResult = searchPage(urlOrigin, i, 1);
	    i = linkCounter;
	    System.out.println(i);
	    for (int a = 0; a < i; a++) {
	    	urlResult = searchPage(urlOrigin, a, 2);
	    	searchPage(urlResult, 30000, 2);
	    	try {
	    		secondPage = urlResult.replace("https://de.wikipedia.org/wiki/", "");
	    	}
			catch (Exception e){}
	    	c = linkCounter;
	    	for (int b = 0; b < c; b++) {
	    		newResult = searchPage(urlResult, b, 3);
	    		try {
	    		    thirdPage = newResult.replace("https://de.wikipedia.org/wiki/", "");
	    		}
	    		catch (Exception e){}
	    				
	    		searchPage(newResult, 30000, 3);	
	    	}
	    }
	    

    }
    
    private static String searchPage(String url, int count, int level) throws IOException {
        //print("Fetching %s...", url);
    	
    	try{
        boolean found = false;
        Document doc = Jsoup.connect(url).get();
        Elements links = doc.select("a[href]");
        linkCounter = 0;
        String lookURL = null;
        //print("\nLinks: (%d)", links.size());
        //System.out.println(url);
        for (Element link : links) {
            lookURL = link.attr("abs:href");
            
            if (count == linkCounter) {
            	return lookURL;
            }
            
            linkCounter++;     
            	
            if (lookURL.equals("https://de.wikipedia.org/wiki/Adolf_Hitler")) {
            	if (level == 1)
            		System.out.println("Hitler detected!: " + firstPage + " ---> Hitler");
            	
            	else if (level == 2)
            		System.out.println("Hitler detected!: " + firstPage + " ---> " + secondPage + " ---> Hitler");
            	
            	else if (level == 3)
            		System.out.println("Hitler detected!: " + firstPage + " ---> " + secondPage + " ---> " + thirdPage + " ---> Hitler");
               	
            	found = true;
            }
            
        }
		
    	}
    	
    	catch (Exception e)
    	{}
    	return null;
    }
    
}
