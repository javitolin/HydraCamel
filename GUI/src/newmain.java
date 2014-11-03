
import java.awt.image.BufferedImage;
import java.io.BufferedInputStream;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.imgproc.Imgproc;


public class newmain {

	private static JFrame frame;
	private static Connection con;
	private static Thread stream;
	private static boolean streaming;
	public static void main1(String[] args)
	{
		frame = new JFrame();
		frame.setVisible(true);
		con = new Connection();
		streaming = false;
		//		debug();
		Scanner scan = new Scanner(System.in);
		System.out.print("> ");
		String input = scan.nextLine();
		while(!input.equals("exit"))
		{
			if(input.equals("connect"))
			{
				if(streaming)
				{
					stream.interrupt();
				}
				ArrayList<String> filters = con.connect();
				for(int i = 0; i < filters.size(); i++)
					System.out.println(filters.get(i));
			}

			else if(input.equals("close"))
			{
				con.close();
			}

			else if(input.length() > 13 && input.substring(0,13).equals("remove_filter"))
			{
				ArrayList<String> filters = con.removeFilter(input.substring(14));
				if(filters == null)
					System.out.println("FILTERS LIST IS NULL...SOMETHING WENT WRONG:\n1.Filter could be in use\n2.Filter could be built-in (like torpedo)");
				else
				{
					for(int i = 0; i < filters.size(); i++)
						System.out.println(filters.get(i));
				}
			}
			
			else if(input.length() > 14 && input.substring(0, 14).equals("add_new_filter"))
			{
				String[] parts = input.split(" ");
				String so_path = parts[1];
				String config_path = parts[2];
				ArrayList<String> filters = con.addFilter(so_path, config_path);
				for(int i = 0; i < filters.size(); i++)
					System.out.println(filters.get(i));
			}

			else if(input.equals("get_hdd_stats"))
			{
				String stats = con.getHDDStats();
				if(stats != null)
					System.out.println(con.getHDDStats());
				else
					System.out.println("STATS IS NULL");
			}
			
			else if(input.length() > 17 && input.substring(0,17).equals("create_new_filter"))
			{
				String[] parts = input.split(" ");
				ArrayList<String> filters;
				if(parts[2].equals("front_camera"))
					filters = con.createFilter(parts[1], true);
				else
					filters = con.createFilter(parts[1], false);
				if(filters != null)
				{
					for(int i = 0; i < filters.size(); i++)
						System.out.println(filters.get(i));
				}
			}

			else if(input.length() > 19 && input.substring(0, 19).equals("chained_filter_list"))
			{
				String[] parts = input.split(" ");
				ArrayList<String> filters = new ArrayList<String>();
				if(!parts[1].equals("nothing"))
					for(int i=1; i<parts.length-1; i++)
					{
						filters.add(parts[i]);
					}

				boolean front;
				if(parts[parts.length-1].equals("front_camera"))
					front = true;
				else
					front = false;
				con.chainedFiltersList(filters, front);
			}

			else if( input.length() > 21 && input.substring(0, 21).equals("unordered_filter_list"))
			{
				String[] parts = input.split(" ");
				ArrayList<String> filters = new ArrayList<String>();
				if(!parts[1].equals("nothing"))
					for(int i=1; i<parts.length-1; i++)
						filters.add(parts[i]);

				boolean front;
				if(parts[parts.length-1].equals("front_camera"))
					front = true;
				else
					front = false;
				con.unorderedFiltersList(filters, front);
			}

			else if(input.equals("end_stream"))
			{
				streaming = false;
				con.endStream();
				stream.interrupt();
			}

			else if(input.length() > 6 && input.substring(0, 6).equals("config"))
			{
				String[] parts = input.split(" ");
				con.changeConfig(parts[1]);
			}

			else if(input.equals("record unfiltered front_camera"))
			{
				con.startRecordingUnfiltered(true);
			}

			else if(input.equals("record unfiltered bottom_camera"))
			{
				con.startRecordingUnfiltered(false);
			}

			else if(input.equals("stopr unfiltered front_camera"))
			{
				con.stopRecordingUnfiltered(true);
			}

			else if(input.equals("stopr unfiltered bottom_camera"))
			{
				con.stopRecordingUnfiltered(false);
			}

			else if(input.length() > 13 && input.substring(0, 13).equals("record_filter"))
			{
				String[] parts = input.split(" ");
				ArrayList<String> filters = new ArrayList<String>();
				for(int i = 1; i < parts.length - 1; i++)
					filters.add(parts[i]);

				if(parts[parts.length-1].equals("true"))
					con.startRecording(filters, true);
				else
					con.startRecording(filters, false);
			}

			else if(input.length() > 14 && input.substring(0, 14).equals("stop_recording"))
			{
				String[] parts = input.split(" ");
				ArrayList<String> filters = new ArrayList<String>();
				for(int i = 1; i < parts.length - 1; i++)
					filters.add(parts[i]);

				if(parts[parts.length-1].equals("true"))
					con.stopRecording(filters, true);
				else
					con.stopRecording(filters, false);
			}

			else if(input.equals("start_video_stream"))
			{
				streaming = true;

				/*
				 * This is the video stream thread. put it where ever you want.
				 */
				stream = new Thread(){
					public void run() {
						//start stream
						con.startStream();
						try{
							//The client sends (for now) any message to the server, and the server starts the video stream.
							DatagramSocket clientSocketVid = new DatagramSocket();
							InetAddress IPAddress = InetAddress.getByName("localhost");
							byte[] sendData = new byte[64*1024];
							byte[] receiveData = new byte[64*1024];
							sendData = "something".getBytes();
							DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, 2014);
							clientSocketVid.send(sendPacket);
							//After the client sent a message, it now receives video stream.
							//each iteration takes care of 1 frame(image) sent from server.
							DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
							while(true)
							{
								clientSocketVid.receive(receivePacket);

								//the image and details about it:
								//First byte is the camera indicator(not exactly).
								//	0 - front camera result
								//	1 - bottom camera result
								//Second&Third bytes are the filter indicator. Second byte is tens , third byte is units
								//rest of bytes are the image
								
								//If filter_ind_tens == 9 and filter_ind_units == 9, then this is an unfiltered image!
								byte[] receivedBytes = receivePacket.getData();

								byte camera_ind = receiveData[0];
								byte filter_ind_tens = receiveData[1];
								byte filter_ind_units = receiveData[2];
								int filter_ind = ((int)filter_ind_tens)*10 + ((int)filter_ind_units); 

								//remove the extra bytes, and get the image!
								byte[] receivedImage = new byte[receivedBytes.length - 3];
								for(int i=3; i<receivedBytes.length; i++)
									receivedImage[i-3] = receivedBytes[i];

								System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
								MatOfByte mob = new MatOfByte(receivedImage);
								//now img ,is the current frame!
								Mat img = Highgui.imdecode(mob, Highgui.IMREAD_UNCHANGED);

								//Now we have the image, which camera it came from, and
								//to which filter it belongs.
								//Note: if chained filters list was used, then this image is filtered by every filter in the list
								//that comes before the image filter(including the image filter)
								//if unordered filters list was used, then this image is filtered by the filter it belongs to.

								//given the filter indicator how can we match the image to its filter "location" in GUI?
								//in the array list sent to the server list.get(filter indicator) will give the wanted filter
								//and from there its the gui people job...

								showResult(img);
							}
						} catch(Exception e){
							e.printStackTrace();
						}
					}


				};
				stream.start();
			}

			else
				System.out.println("Try again...");

			System.out.println();
			System.out.println("> ");
			input = scan.nextLine();
		}		
	}

	public static void debug()
	{
		con.connect();

		ArrayList<String> filters = new ArrayList<String>();
		filters.add("torpedo");
		filters.add("path");
		con.chainedFiltersList(filters, true);
		//		con.createFilter("eliran", true);

		//		filters.clear();
		//		filters.add("torpedo");
		//		con.unorderedFiltersList(filters, true);

		streaming = true;

		/*
		 * This is the video stream thread. put it where ever you want.
		 */
		stream = new Thread(){
			public void run() {
				//start stream
				con.startStream();
				try{
					//The client sends (for now) any message to the server, and the server starts the video stream.
					DatagramSocket clientSocketVid = new DatagramSocket();
					InetAddress IPAddress = InetAddress.getByName("localhost");
					byte[] sendData = new byte[64*1024];
					byte[] receiveData = new byte[64*1024];
					sendData = "something".getBytes();
					DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, 2014);
					clientSocketVid.send(sendPacket);
					//After the client sent a message, it now receives video stream.
					//each iteration takes care of 1 frame(image) sent from server.
					DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
					while(true)
					{
						clientSocketVid.receive(receivePacket);

						//the image and details about it:
						//First byte is the camera indicator(not exactly).
						//	0 - front camera result
						//	1 - bottom camera result
						//	2 - unfiltered image
						//Second&Third bytes are the filter indicator. Second byte is tens , third byte is units
						//rest of bytes are the image
						byte[] receivedBytes = receivePacket.getData();

						byte camera_ind = receiveData[0];
						byte filter_ind_tens = receiveData[1];
						byte filter_ind_units = receiveData[2];
						int filter_ind = ((int)filter_ind_tens)*10 + ((int)filter_ind_units); 

						//remove the extra bytes, and get the image!
						byte[] receivedImage = new byte[receivedBytes.length - 3];
						for(int i=3; i<receivedBytes.length; i++)
							receivedImage[i-3] = receivedBytes[i];

						System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
						MatOfByte mob = new MatOfByte(receivedImage);
						//now img ,is the current frame!
						Mat img = Highgui.imdecode(mob, Highgui.IMREAD_UNCHANGED);

						//Now we have the image, which camera it came from, and
						//to which filter it belongs.
						//Note: if chained filters list was used, then this image is filtered by every filter in the list
						//that comes before the image filter(including the image filter)
						//if unordered filters list was used, then this image is filtered by the filter it belongs to.

						//given the filter indicator how can we match the image to its filter "location" in GUI?
						//in the array list sent to the server list.get(filter indicator) will give the wanted filter
						//and from there its the gui people job...

						showResult(img);
					}
				} catch(Exception e){
					e.printStackTrace();
				}
			}


		};
		stream.start();

		filters.clear();
		filters.add("eliran");
		con.unorderedFiltersList(filters, true);
		//		OMG!
	}

	public static void showResult(Mat img) 
	{
		Imgproc.resize(img, img, new Size(320, 240));
		MatOfByte matOfByte = new MatOfByte();
		Highgui.imencode(".jpg", img, matOfByte);
		byte[] byteArray = matOfByte.toArray();
		BufferedImage bufImage = null;
		try {
			InputStream in = new ByteArrayInputStream(byteArray);
			bufImage = ImageIO.read(in);
			frame.getContentPane().removeAll();
			frame.getContentPane().add(new JLabel(new ImageIcon(bufImage)));
			frame.pack();
			frame.revalidate();
			frame.repaint();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
