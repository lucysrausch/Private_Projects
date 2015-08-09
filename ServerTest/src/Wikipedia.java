import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URL;
import java.nio.channels.Channels;
import java.nio.channels.ReadableByteChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.jsoup.Jsoup;
import org.jsoup.nodes.Document;
import org.jsoup.nodes.Element;
import org.jsoup.select.Elements;




public class Wikipedia {
	public static void main(String[] args) throws Exception {
		HashMap<String, ArrayList<String>> hashmap = 
		        new HashMap<String, ArrayList<String>>();
		
            int nameCounter = 0;		
		
		    ArrayList<String> arraylist = new ArrayList<String>();
		    arraylist.add("Hello");
		    arraylist.add("World.");
		    
		    //arraylist.clear();
		    hashmap.put("my key", arraylist);

		    ArrayList<String> outputList = hashmap.get("my key");
		    System.out.println(outputList);
		    
		   // ArrayList<String> arraylist = new ArrayList<String>();
		    arraylist.add("Hello");
		    arraylist.add("World.");
		    
			
		}
		
	}