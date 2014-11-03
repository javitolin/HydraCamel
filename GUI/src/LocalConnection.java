

import java.io.*;
import java.net.*;
import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.highgui.Highgui;

public class LocalConnection {

	private Socket tcpSocket;
	private DataOutputStream outToServer;
	//private BufferedInputStream inFromServer;
	private DataInputStream inFromServer;
	public boolean streaming;
	private boolean connected;

	public LocalConnection()
	{
		connected = false;
		streaming = false;
	}

	/**
	 * Connects to the server
	 * @return List of filters that are in the machine
	 */
	public ArrayList<String> connect()
	{

		try {
			this.tcpSocket = new Socket("localhost", 5000);
			this.outToServer = new DataOutputStream( tcpSocket.getOutputStream() );
			//			this.inFromServer = new BufferedInputStream(tcpSocket.getInputStream());
			this.inFromServer = new DataInputStream(tcpSocket.getInputStream());
			connected = true;
			return getFilters();
		
		} catch (UnknownHostException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		return null;

		/*
		connected = true;
		 ArrayList<String> list = new  ArrayList<String>();
		 list.add("loc1");
		 list.add("loc2");
		 return list;

		 */
	}

	/**
	 * @return true if the socket is currently connected, false otherwise
	 */
	public boolean isConnected()
	{
		return connected;
	}

	/**
	 * Close the connection to the server.
	 * Use this function when existing the program!
	 */
	public void close()
	{
		if(streaming)
			endStream();

		sendString("106");
		try 
		{
			tcpSocket.close();
		} catch (IOException e) {
			e.printStackTrace();
		}

		connected = false;
	}

	/**
	 * Sends an image path to a local server which filter the image, and returns the output.
	 * @param path - image path
	 * @return on success - byte array representing the image. on failure - null.
	 */
	public ArrayList<Mat> filterImage(String path)
	{
		String temp = "";
		byte mat_arr[];
		boolean error = false;

		//Send data
		sendString("115");
		sendString(zeroWraper(Integer.toString(path.length()), 10));
		sendString(path);

		ArrayList<Mat> mats = new ArrayList<Mat>();
		//Receive data
		try
		{
			//Receive number of filtered images
			for(int i = 0; i < 2 && !error; i++)
			{
				temp += (char)inFromServer.read();
				if(temp.equals("e"))
					error = true;
			}
			if(!error)
			{
				int filtered_images = Integer.parseInt(temp);

				for(int j = 0; j < filtered_images; j++)
				{
					error = false;
					temp = "";
					//Receive image size
					for(int i=0; i<10 && !error; i++)
					{
						temp += (char)inFromServer.read();
						if(temp.equals("e"))
						{
							error = true;
						}
					}
					if(error)
						continue;

					//Receive image
					int mat_size = Integer.parseInt(temp);
					System.out.println("Receving: " + mat_size + " bytes for image");
					mat_arr = new byte[mat_size];
					for(int i=0; i < mat_size && !error; i++)
					{
						int read = (byte)inFromServer.read();
						mat_arr[i] = (byte)read;
						//				if(read == -1) Not sure if needed
						//					error = true;
					}
					MatOfByte mob = new MatOfByte(mat_arr);
					Mat img = Highgui.imdecode(mob, Highgui.IMREAD_UNCHANGED);
					mats.add(img);
				}
				if(mats.size() == filtered_images)
					return mats;
				else
					return null;
			}
			else
				return null;
		} catch(Exception e) {
			e.printStackTrace();
			return null;
		}
	}

	/**
	 * Creating a new filter using the list currently used by the server.
	 * If frontCamera is true, then the front filters list is used, otherwise the bottom filters list is used.
	 * @param name - name of the filter
	 * @return Current list of filters in the server on success, null otherwise
	 */
	public ArrayList<String> createFilter(String name)
	{
		sendString("113");

		if(getError())//The filter list in the server is empty
		{
			return null;
		}
		else
		{
			//Sending name size
			sendString(zeroWraper(Integer.toString(name.length()), 2));

			//Sending name
			sendString(name);

			if(getError()) //name already taken
			{
				return null;
			}
			else //All good
			{
				if(getError()) //there was some problem loading the new filter. server is rolling back
					return null;
				else
					return getFilters();
			}
		}
	}

	/**
	 * Uploads a new filter to the server dynamically.
	 * @param soPath - Path to the .so file
	 * @param configPath - Path to the .config file
	 * @return Current list of filters in the server on success, null otherwise
	 */
	public ArrayList<String> addFilter(String soPath, String configPath)
	{
		sendString("105");

		if(sendFileName(soPath))
		{
			sendFile(soPath);
			sendFile(configPath);
		}
		else
			System.out.println("File name too long!");

		sendString("105");
		boolean error = false;
		if(sendFileName(soPath))
		{
			error = getError();
			if(!error)
			{
				sendFile(soPath);
				sendFile(configPath);
			}
		}
		else
		{
			error = true;
			System.out.println("File name too long!");
		}

		if(error)
			return null;
		return getFilters();
	}

	/**
	 * Tells the server to use chained filter list.
	 * If the operation failed, the server roll back every change made by this function.
	 * @param list - List of names
	 * @return false on success, true on failure
	 */
	public boolean chainedFiltersList(ArrayList<String> list)
	{
		sendString("104");

		//Sending number of filters
		sendString( zeroWraper(Integer.toString(list.size()), 2));

		//For each filter sending its name size and name
		for(int i = 0; i < list.size(); i++)
		{
			//Sending name size
			sendString( zeroWraper(Integer.toString(list.get(i).length()), 2) );
			//Send filter name
			sendString( list.get(i) );
		}

		return getError();
	}

	/**
	 * Tells the server to use unordered filter list.
	 * If the operation failed, the server roll back every change made by this function.
	 * @param list - List of names
	 * @return false on success, true on failure
	 */
	public boolean unorderedFiltersList(ArrayList<String> list)
	{
		sendString("103");

		//Sending number of filters
		sendString( zeroWraper(Integer.toString(list.size()), 2));

		//For each filter sending its name size and name
		for(int i = 0; i < list.size(); i++)
		{
			//Sending name size
			sendString( zeroWraper(Integer.toString(list.get(i).length()), 2) );
			//Send filter name
			sendString( list.get(i) );
		}

		return getError();
	}

	/**
	 * Ends the video stream
	 */
	public void endStream()
	{
		sendString("102");
		this.streaming = false;
	}

	/**
	 * Starts the video stream.
	 * After calling this function, the server awaits for a client to initiate contact.
	 * @param videoPath - path to the video file. Path must be absolute and not relative!
	 */
	public void startStream(String videoPath)
	{
		sendString("101");
		sendString(zeroWraper(Integer.toString(videoPath.length()), 10));
		sendString(videoPath);
		this.streaming = true;
	}

	/**
	 * Sends the server a config file
	 * @param path - Path to the file
	 */
	public void changeConfig(String path)
	{
		//Sending change config server code
		sendString("100");
		if(sendFileName(path))
			sendFile(path);
		else
			System.out.println("File name too long!");
	}

	/**
	 * Get a list of filters to record and sends them to the server.
	 * Note: This function doesn't make the server RUN those filters.
	 * 	First tell the server to run those and then call this function!
	 * @param filters - list of filters to record
	 * @return false on success, true on failure
	 */
	public boolean startRecording(ArrayList<String> filters)
	{
		sendString("107");

		//Sending number of filters
		sendString( zeroWraper(Integer.toString(filters.size()), 2));

		//For each filter sending its name size and name
		for(int i = 0; i < filters.size(); i++)
		{
			//Sending name size
			sendString( zeroWraper(Integer.toString(filters.get(i).length()), 2) );
			//Send filter name
			sendString( filters.get(i) );
		}

		return getError();
	}

	/**
	 * Get a list of filters and sends them to the server.
	 * Note: This function doesn't make the server STOP running those filters. 
	 * @param filters - list of filters to stop recording
	 * @return false on success, true on failure
	 */
	public boolean stopRecording(ArrayList<String> filters)
	{
		sendString("108");

		//Sending number of filters
		sendString( zeroWraper(Integer.toString(filters.size()), 2));

		//For each filter sending its name size and name
		for(int i = 0; i < filters.size(); i++)
		{
			//Sending name size
			sendString( zeroWraper(Integer.toString(filters.get(i).length()), 2) );
			//Send filter name
			sendString( filters.get(i) );
		}

		return getError();
	}

	/**
	 * @return String representing the submarine's hard disk stats on success, null otherwise.
	 */
	public String getHDDStats()
	{
		sendString("109");
		String temp = "";
		String stats = "";

		if(getError()) //Could not retreive stats on the server side
		{
			return null;
		}
		try
		{
			for(int i=0; i<10; i++)
				temp += (char)inFromServer.read();

			int stats_size = Integer.parseInt(temp);
			for(int i=0; i<stats_size; i++)
				stats += (char)inFromServer.read();
		} catch(Exception e) {
			close();
			e.printStackTrace();
		}

		return stats;
	}

	/**
	 * removes a given filter from the server
	 * @param name - name of the filters
	 * @return - List of filters in the machine on success, null on failure
	 */
	public ArrayList<String> removeFilter(String name)
	{
		sendString("112");
		sendString(zeroWraper(Integer.toString(name.length()), 2));
		sendString(name);
		if(getError()) //Filter is currently is use
		{
			return null;
		}
		else
		{
			if(getError()) //Tried to remove built-in or non-existence filter
				return null;
			else
				return getFilters();
		}
	}

	/**
	 * @return true if an error occurred, false otherwise.
	 */
	private boolean getError()
	{
		try {
			char error = (char) inFromServer.read();
			if(error == '1')
				return true;
			else
				return false;
		} catch(Exception e) {
			close();
			e.printStackTrace();
			return true;
		}
	}

	/**
	 * @return List of filters in the machine
	 */
	private ArrayList<String> getFilters()
	{
		String temp = "";
		ArrayList<String> filters = new ArrayList<String>();
		try
		{
			//get number of filters
			for(int i=0; i<2; i++)
				temp += (char) inFromServer.read();

			int num_of_filters = Integer.parseInt(temp);
			int filter_name_size;
			for(int i = 0; i < num_of_filters; i++)
			{
				temp = "";
				for(int j = 0; j < 2; j++)
					temp += (char) inFromServer.read();

				filter_name_size = Integer.parseInt(temp);
				temp = "";
				for(int j = 0; j < filter_name_size; j++)
					temp += (char) inFromServer.read();

				filters.add(temp);
			}

		} catch(Exception e) {
			close();
			e.printStackTrace();
		}

		return filters;
	}

	/**
	 * Sends the given string to the server
	 * @param str - string to be sent
	 */
	private void sendString(String str)
	{
		try 
		{
			this.outToServer.writeBytes(str);
		} catch (IOException e) {
				close();
			e.printStackTrace();
		}
	}

	/**
	 * Sends the name length, and then sends the name
	 * @param path - path to the file
	 * @return true on success, false otherwise.
	 */
	private boolean sendFileName(String path)
	{
		File file = new File(path);
		String file_name = file.getName();
		if(file_name.length() < 100)
		{
			sendString(zeroWraper(Integer.toString(file_name.length()), 2));
			sendString(file_name);
			return true;
		}
		else
			return false;
	}

	/**
	 * Sends the file length, and the sends the file
	 * @param path - path to the file
	 */
	private void sendFile(String path)
	{
		try {
			long time = System.currentTimeMillis();
			FileInputStream fileInputStream = new FileInputStream(path);
			sendString(zeroWraper(Long.toString(fileInputStream.getChannel().size()), 10));

			byte[] buffer = new byte[64*1024];
			int bytesRead = 0;
			long totalSent = 0;

			while ( (bytesRead = fileInputStream.read(buffer)) != -1)
			{
				if (bytesRead > 0)
				{   
					this.outToServer.write(buffer, 0, bytesRead);
					totalSent += bytesRead;
				}   
			}
			System.out.println("Sent " + totalSent + " bytes in " + (System.currentTimeMillis() - time) + "ms.");
			fileInputStream.close();

		} catch (FileNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			close();
			e.printStackTrace();
		}
	}

	/**
	 * Adding leading zeros to a given number.
	 * @param num - number to add leading zeros to.
	 * @param requestedLength - The wanted number of bytes of the number
	 * @return A new number with leading zeros
	 */
	private String zeroWraper(String num, int requestedLength)
	{
		while( requestedLength > num.length() )
			num = "0" + num;

		return num;
	}
}
