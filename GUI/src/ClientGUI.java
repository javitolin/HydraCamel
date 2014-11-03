

import java.io.File;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Event;
import java.awt.Image;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintWriter;
import java.io.FileWriter;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import javax.imageio.ImageIO;
import javax.swing.BorderFactory;
import javax.swing.DefaultListModel;
import javax.swing.Icon;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JEditorPane;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTree;
import javax.swing.ListSelectionModel;
import javax.swing.WindowConstants;
import javax.swing.border.EtchedBorder;
import javax.swing.border.TitledBorder;
import javax.swing.event.ListSelectionEvent;

import javax.swing.event.ListSelectionListener;
import javax.swing.event.TreeSelectionEvent;
import javax.swing.event.TreeSelectionListener;
import javax.swing.plaf.basic.BasicArrowButton;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeCellRenderer;
import javax.swing.tree.DefaultTreeModel;
import javax.swing.tree.TreeSelectionModel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;




public class ClientGUI implements ActionListener, TreeSelectionListener{

	JFrame frame;
	private Connection con = new Connection();
	private LocalConnection localCon = new LocalConnection();
	public Thread camStream;
	public Thread videoStream;
	public String videoPath;

	ConfigWindow configWindow = null;

	private Menu menu;

	public JPanel unfilteredStreamCam1, filteredStreamCam1;
	public JPanel unfilteredStreamCam2, filteredStreamCam2;
	public JPanel unfilteredVideoStream, filteredVideoStream;

	
	// 0 -> config connection file, 1 -> config local connection file, 
	int conConfigOpen = -1;
	
	//indicator for filtered images to show. indicates the filter number.
	int fCamImage = -1, bCamImage = -1,  VideoImage = -1;
	//indicator chained or not. 0 -> chained, 1 -> unchained
	//int fCamChained = -1, bCamChained = -1,  VideoImageChained = -1;


	public JButton configCam;

	public DefaultListModel<String> consoleListModel;

	//contains the names of all filters in sub server
	public ArrayList<String> allFiltersListSub = new ArrayList<String>();
	//contains the names of all algorithms in sub server
	public ArrayList<String> allAlgorithmsListSub = new ArrayList<String>();

	//contains the names of all filters in local server
	public ArrayList<String> allFiltersListLoc = new ArrayList<String>();
	//contains the names of all algorithms in local server
	public ArrayList<String> allAlgorithmsListLoc = new ArrayList<String>();

	//contains the names of filters that user want to run them
	public ArrayList<String> currFiltersListCam1 = new ArrayList<String>();
	public ArrayList<String> currFiltersListCam2 = new ArrayList<String>();
	public ArrayList<String> currFiltersListVideo = new ArrayList<String>();

	//indicator for filter or algorithm. 0 - filter is run, 1 - algorithm is run
	int fCFilterRunning = -1, bCFilterRunning = -1, videoFilterRunning = -1; 

	//tree lists 
	public DefaultTreeModel model;
	public JTree tree;
	public DefaultMutableTreeNode root;
	public DefaultMutableTreeNode frontCamList;
	public DefaultMutableTreeNode bottomCamList;
	public DefaultMutableTreeNode videoList;

	ArrayList<Mat> filteredImages;

	public void startGUI() throws IOException{	

		initGui();
		menu.setGui(this);
		menu.disableSubServerButtons();
		menu.disableLocalServerButtons();


		//		for(int i = 0; i < allAlgorithmsList.size(); i++){
		//			System.out.println(i + " " + allAlgorithmsList.get(i));
		//		}
		//		for(int i = 0; i < allFiltersList.size(); i++){
		//			System.out.println(i + " " + allFiltersList.get(i));
		//		} 
	}
	/////////////////////////////////////////////////////////             Video thread   /////////////////////////////////////////
	public void initVideoThread(final String path){
		videoStream = new Thread(){
			public void run() {
				//start stream
				localCon.startStream(path);
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
						
						if(menu.fileOpen == 1){//the video file is open
							if(filter_ind == 99)
								showImage(img, 1, -1, 0); //unfiltered

							else
								if(filter_ind == VideoImage)   
									showImage(img, 1, -1, 1); //filtered 
						}
					}
				} catch(Exception e){
					e.printStackTrace();
				}
			}
		};
		videoStream.start();
	}
	////////////////////////////////////////////////   End video stream   //////////////////////////////////////////////////

	public void initGui(){

		frame = new JFrame();
		
		frame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
		  frame.addWindowListener(new WindowAdapter() {
		     public void windowClosing(WindowEvent ev) {
		      boolean close = true;
		      if(con.isConnected())
		    	  con.close();
		      if(localCon.isConnected())
		    	  localCon.close();			
		      if (close) {
		          frame.dispose();
		          System.exit(0);
		         }
		       }            
		});
		
		frame.setSize(1300,730);
		frame.setLayout(null);

		//	frame.setContentPane(new JLabel(menu.createImageIcon("background1.png")));    

		menu = new Menu();

		frame.setJMenuBar(menu.initMenuBar());

		// Camera1: filtered and unfiltered panel 
		unfilteredStreamCam1 = new JPanel();
		unfilteredStreamCam1.setBounds(245, 35,320, 240);
		unfilteredStreamCam1.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.gray), null), "Front Camera: Unfiltered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
		frame.add(unfilteredStreamCam1);

		filteredStreamCam1 = new JPanel();
		filteredStreamCam1.setBounds(245,320,320, 240);
		filteredStreamCam1.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.gray), null), "Front Camera: Filtered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
		frame.add(filteredStreamCam1);

		// Camera2: filtered and unfiltered panel 
		unfilteredStreamCam2 = new JPanel();
		unfilteredStreamCam2.setBounds(595, 35, 320, 240);
		unfilteredStreamCam2.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.gray), null), "Bottom Camera: Unfiltered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
		frame.add(unfilteredStreamCam2);

		filteredStreamCam2 = new JPanel();
		filteredStreamCam2.setBounds(595, 320, 320, 240);
		filteredStreamCam2.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.gray), null), "Bottom Camera: Filtered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
		frame.add(filteredStreamCam2);

		// Video and image: filtered and unfiltered panel 
		unfilteredVideoStream = new JPanel();
		unfilteredVideoStream.setBounds(945, 35, 320, 240);
		unfilteredVideoStream.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.gray), null), "Video or Image: Unfiltered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
		frame.add(unfilteredVideoStream);

		filteredVideoStream = new JPanel();
		filteredVideoStream.setBounds(945, 320, 320, 240);
		filteredVideoStream.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.gray), null), "Video or Image: Filtered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
		frame.add(filteredVideoStream);


		//current filters list
		root = new DefaultMutableTreeNode("Current Filters:");
		frontCamList = new DefaultMutableTreeNode("Front Camera:");
		bottomCamList = new DefaultMutableTreeNode("Bottom Camera:");
		videoList = new DefaultMutableTreeNode("Video or Image:");

		tree = new JTree(root);
		tree.getSelectionModel().setSelectionMode(TreeSelectionModel.SINGLE_TREE_SELECTION);
		tree.addTreeSelectionListener(this);

		model = (DefaultTreeModel)tree.getModel();
		root = (DefaultMutableTreeNode)model.getRoot();

		root.add(frontCamList);
		root.add(bottomCamList);
		root.add(videoList);

		model.reload(root);
		// Remove default JTree icons
		DefaultTreeCellRenderer renderer = (DefaultTreeCellRenderer) tree.getCellRenderer();
		renderer.setLeafIcon(null);
		renderer.setClosedIcon(null);
		renderer.setOpenIcon(null); 
		JScrollPane pane = new JScrollPane(tree);
		pane.setPreferredSize(new Dimension(180, 440));
		JPanel p = new JPanel();
		p.add(pane);
		p.setPreferredSize(new Dimension(180, 450));
		p.setBounds(20, 38, 180, 450);	    
		frame.add(p);

		//setting button
		ImageIcon set = menu.createImageIcon("Set.gif");
		configCam = new JButton(set);
		//make button transparently
		configCam.setOpaque(false);
		configCam.setContentAreaFilled(false);
		configCam.setBorderPainted(false);

		configCam.setBounds(202, 43, 25, 25);
		frame.add(configCam);
		configCam.addActionListener(this);

		//console: for error and messages for user
		consoleListModel = new DefaultListModel<String>();
		JList<String> list = new JList<String>(consoleListModel);
		JPanel filterListPanel = new JPanel();  

		JScrollPane listScroller = new JScrollPane(list);
		listScroller.setPreferredSize(new Dimension(1250, 100));
		filterListPanel.setBounds(20, 550, 1250, 105);
		filterListPanel.add(listScroller);
		frame.add(filterListPanel);

		TitledBorder title = new TitledBorder(new EtchedBorder(), "Console:",TitledBorder.LEFT, TitledBorder.LEFT);
		listScroller.setBorder(title);

		frame.setVisible(true);	



		//TODO:: remove next  lines
		//	System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		//   	Mat frame = Highgui.imread("C:\\Users\\user\\workspace1\\StupidClient\\kenes.png");
		// get a new frame from camera
		//   System.out.println("frame = " + frame.empty());
		//	showImage(frame, 0, 0, 0);

		//	Mat frame2 = Highgui.imread("C:\\Users\\user\\workspace1\\StupidClient\\kenes2.png");
		//   showImage(frame2, 0, 0, 1);

	}

	public Connection getConnection(){
		return this.con;
	}

	public LocalConnection getLocalConnection(){
		return this.localCon;
	}

	public void actionPerformed(ActionEvent e) {

		if(e.getSource() == configCam){
			DefaultMutableTreeNode node = (DefaultMutableTreeNode)tree.getLastSelectedPathComponent();
			if (node == null) 
				return;
			if (node.isLeaf() && !root.isNodeChild(node)) {
				//get selected filter name
				//System.out.println(node.getUserObject());
				 String path = (String) node.getUserObject();
				
				 File configFile = new File(System.getProperty("user.dir")+"/" + path + ".config");
				
				 if(!configFile.isFile()){
					 consoleListModel.addElement("Cannot open configuration file. Check that there is a file " + System.getProperty("user.dir")+"/" + path + ".config");
					 return;
				}
			     try {
			    	 configWindow = new ConfigWindow(path, this);
			     } catch (IOException e1) {
			    	 e1.printStackTrace();
			     }   

				configWindow.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
				configWindow.setTitle("Configuration");
				configWindow.pack();		
				configWindow.setResizable(false);
				configWindow.setAlwaysOnTop(true);
				configWindow.setVisible(true);
			//configWindow.addWindowListener(new Event());  
			}
		}
		else{
			consoleListModel.addElement("There is no selected filter");
		}

	}

	public void closeConfigWindow(){
		configWindow.dispose();  
	}

	@Override
	public void valueChanged(TreeSelectionEvent arg0) {

		DefaultMutableTreeNode node = (DefaultMutableTreeNode)tree.getLastSelectedPathComponent();
		if (node == null) 
			return;
		Object nodeInfo = node.getUserObject();     
		if (node.isLeaf()) {
			int fcamindex = this.frontCamList.getIndex(node);
			int bcamindex = this.bottomCamList.getIndex(node);
			int videoindex = this.videoList.getIndex(node);

			if(fcamindex != -1){
				if(fCamImage != -1){
					fCamImage = fcamindex;
					consoleListModel.addElement("Displayed " + node.toString()+ " filtered image.");
				}
				else {
					consoleListModel.addElement("Run filters or algorithms in order to see filtered image.");
				}
				if(fCFilterRunning == 0)
					configCam.setVisible(true);
				else
					configCam.setVisible(false);
				conConfigOpen = 0;
			}
			if(bcamindex != -1){
				if(bCamImage != -1){
					bCamImage = bcamindex;
					consoleListModel.addElement("Displayed " + node.toString()+ " filtered image.");
				}
				else {
					consoleListModel.addElement("Run filters or algorithms in order to see filtered image.");
				}
				if(bCFilterRunning == 0)
					configCam.setVisible(true);
				else
					configCam.setVisible(false);
				conConfigOpen = 0;
			}
			if(videoindex != -1){
				if(VideoImage != -1){
					VideoImage = videoindex;
					if(menu.fileOpen == 0 && filteredImages != null && filteredImages.size() > VideoImage){
						//display filtered image
						showImage(filteredImages.get(VideoImage), 1, -1, 1);
					}
					consoleListModel.addElement("Displayed " + node.toString()+ " filtered image.");
				}
				else {
					consoleListModel.addElement("Run filters or algorithms in order to see filtered image.");
				}
				if(videoFilterRunning == 0)
					configCam.setVisible(true);
				else
					configCam.setVisible(false);
				conConfigOpen = 1;
			}

		} 
	}

	/*
	 * camOrFile: 0 - camera, 1 - file
	 * cam indicator: 0 - front, 1 - bottom
	 * filtered: 0 - unfiltered, 1 - filtered 
	 */
	

	public void showImage(Mat img, int camOrFile, int cam, int filtered) {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		Imgproc.resize(img, img, new Size(320, 240));
		MatOfByte matOfByte = new MatOfByte();
		Highgui.imencode(".jpg", img, matOfByte);
		byte[] byteArray = matOfByte.toArray();
		BufferedImage bufImage = null;
		try {
			InputStream in = new ByteArrayInputStream(byteArray);
			bufImage = ImageIO.read(in);
			if(bufImage == null)
				System.out.println("Not suppose to happen...");    
			else{
				//file
				if(camOrFile == 1){
					//unfiltered
					if(filtered == 0){
						unfilteredVideoStream.removeAll();
						unfilteredVideoStream.add(new JLabel(new ImageIcon(bufImage)));
						unfilteredVideoStream.revalidate();
						unfilteredVideoStream.repaint();				
					}
					//filtered
					else if(filtered == 1){
						filteredVideoStream.removeAll();
						filteredVideoStream.add(new JLabel(new ImageIcon(bufImage)));
						filteredVideoStream.revalidate();
						filteredVideoStream.repaint();
					}
				}
				//cam
				else if (camOrFile == 0)
				{
					//front camera
					if(cam == 0){						
						//unfiltered
						if(filtered == 0){
							unfilteredStreamCam1.removeAll();
							unfilteredStreamCam1.add(new JLabel(new ImageIcon(bufImage)));
							unfilteredStreamCam1.revalidate();
							unfilteredStreamCam1.repaint();
						}
						//filtered
						else if(filtered == 1){
							filteredStreamCam1.removeAll();
							filteredStreamCam1.add(new JLabel(new ImageIcon(bufImage)));
							filteredStreamCam1.revalidate();
							filteredStreamCam1.repaint();
						}
					}
					//bottom camera
					else if(cam == 1){
						//unfiltered
						if(filtered == 0){
							unfilteredStreamCam2.removeAll();
							unfilteredStreamCam2.add(new JLabel(new ImageIcon(bufImage)));
							unfilteredStreamCam2.revalidate();
							unfilteredStreamCam2.repaint();

						}
						//filtered
						else if(filtered == 1){
							filteredStreamCam2.removeAll();
							filteredStreamCam2.add(new JLabel(new ImageIcon(bufImage)));
							filteredStreamCam2.revalidate();
							filteredStreamCam2.repaint();								
						}
					}
				}
			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
		public void startStreamThread(){
		camStream = new Thread(){
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

						//front cam, filtered and image is for chosen filter to show 
						if((camera_ind == 0) && (filter_ind != 99) && (fCamImage == filter_ind)){
							showImage(img, 0, 0, 1);
						}
						//bottom cam, filtered and image is for chosen filter to show
						else if(((camera_ind == 1) && (filter_ind != 99) && (bCamImage == filter_ind))){
							showImage(img, 0, 1, 1);
						}
						//unfiltered
						else if(filter_ind == 99){
							showImage(img, 0, camera_ind, 0); 
						}
					}
				} catch(Exception e){
					e.printStackTrace();
				}
			}
		}; 
		camStream.start();
	}
}