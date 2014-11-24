

import java.awt.*; 
import java.awt.event.*;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JMenu;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JRadioButton;
import javax.swing.JRadioButtonMenuItem;
import javax.swing.ButtonGroup;
import javax.swing.JMenuBar;
import javax.swing.JTextField;
import javax.swing.ImageIcon;
import javax.swing.JPanel;
import javax.swing.JTextArea;
import javax.swing.JScrollPane;
import javax.swing.JFrame;
import javax.swing.border.EtchedBorder;
import javax.swing.border.TitledBorder;
import javax.swing.event.MenuEvent;
import javax.swing.event.MenuListener;
import javax.swing.filechooser.FileNameExtensionFilter;
import javax.swing.plaf.basic.BasicArrowButton;
import javax.swing.tree.DefaultMutableTreeNode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.highgui.Highgui;

public class Menu extends JPanel implements ActionListener {

	private MyMenuItem exit;
	private JMenuItem addSub, createSub, chooseFiltersSub, chooseAlgosSub;
	private JMenuItem addLoc, createLoc, chooseFiltersLoc, chooseAlgosLoc;
	private ClientGUI gui;
	private Image openImg;
	private String filterPath, configPath;
	private JButton openFilterButton, openConfigButton;
	private JTextField filterW, configW;

	private JMenuItem runCFrontCamera, runCBottomCamera, runCVideoImage;
	private JMenuItem runUCFrontCamera, runUCBottomCamera, runUCVideoImage;
	//unfiltered cam
	private JMenuItem saveUFCamera, saveUBCamera, stopSaveUFCamera, stopSaveUBCamera;
	//filtered cam
	private JMenuItem saveFFCamera, saveFBCamera, stopSaveFFCamera, stopSaveFBCamera;
	private JMenuItem saveFVideoOrImage, stopSaveFVideoOrImage;
	//indicator for recording
	int fFCam = 0, uFCam = 0, fBCam = 0, uBCam = 0, videoImg = 0; 

	private JMenuItem openImage, openVideo;
	//indicate wich file is open, 0 - image, 1 - video
	int fileOpen = -1;
	String pathToLastFile = "";


	//connection to servers
	private JMenuItem subServerConnect, localServerConnect;

	//connection to cameras
	private JMenuItem camStartStream, camStopStream, closeVideo;

	private JMenuItem removeFiltersSub, removeAlgSub;  
	private JMenuItem removeFiltersLoc, removeAlgLoc; 
	private JMenuItem statisticSubServer, statisticLocalServer;

	//show the names of all filters, get if from gui all the time
	private DefaultListModel<String> lListModel;
	//list that contains lListModel elements
	private JList<String> lList;
	//show the names of chosen filters to run
	private DefaultListModel<String> rListModel;
	//list that contains RListModel elements
	private JList<String> rList;
	//arrows - addButton and removeButton: add and remove filters from current filters list
	private BasicArrowButton addButton;
	private BasicArrowButton removeButton;

	JFileChooser fc;

	public JMenuBar initMenuBar() {

		JMenuBar menuBar = new JMenuBar();
		JMenu menu, submenu, submenu2;

		///////////////   parameters initialization ///////////////////
		//openImage 
		try {
			openImg = ImageIO.read(getClass().getResource("Open16.gif"));
		} catch (IOException ex) {
			//TODO:: error
		}

		openFilterButton = new JButton();
		openConfigButton = new JButton();

		openFilterButton.setIcon(new ImageIcon(openImg));
		openFilterButton.setBorder(null);
		openConfigButton.setIcon(new ImageIcon(openImg));
		openConfigButton.setBorder(null);

		openFilterButton.addActionListener(this);
		openConfigButton.addActionListener(this);

		//////////////////////////////////////////////////  Connection         ////////////////////////

		menu = new JMenu("Connection");
		menu.setMnemonic(KeyEvent.VK_N);
		menuBar.add(menu);

		//add new filter
		subServerConnect = new JMenuItem("Connect to Sub Server");
		subServerConnect.addActionListener(this);
		menu.add(subServerConnect);

		//add new filter
		localServerConnect = new JMenuItem("Connect to Local Server", KeyEvent.VK_T);
		localServerConnect.addActionListener(this);
		menu.add(localServerConnect);

		//////////////////////////////////////////////////   Filter         ////////////////////////

		menu = new JMenu("Filter");
		menu.setMnemonic(KeyEvent.VK_N);
		menuBar.add(menu);

		//add new filter in sub
		addSub = new JMenuItem("Add in Sub", KeyEvent.VK_T);
		addSub.addActionListener(this);
		menu.add(addSub);

		//add new filter
		addLoc = new JMenuItem("Add locally", KeyEvent.VK_T);
		addLoc.addActionListener(this);
		menu.add(addLoc);

		// create new filter from already existing filters in sub
		createSub = new JMenuItem("Save as in Sub (New Algorithm)", createImageIcon("Save16.gif"));
		createSub.setMnemonic(KeyEvent.VK_R);
		createSub.addActionListener(this);
		menu.add(createSub);

		// create new filter from already existing filters locally
		createLoc = new JMenuItem("Save as locally (New Algorithm)", createImageIcon("Save16.gif"));
		createLoc.setMnemonic(KeyEvent.VK_R);
		createLoc.addActionListener(this);
		menu.add(createLoc);

		// select filters to run in sub
		chooseFiltersSub = new JMenuItem("Select in Sub", KeyEvent.VK_T);
		chooseFiltersSub.addActionListener(this);
		menu.add(chooseFiltersSub);

		// select filters to run locally
		chooseFiltersLoc = new JMenuItem("Select locally", KeyEvent.VK_T);
		chooseFiltersLoc.addActionListener(this);
		menu.add(chooseFiltersLoc);


		// remove filters in sub
		removeFiltersSub = new JMenuItem("Remove in Sub", KeyEvent.VK_T);
		removeFiltersSub.addActionListener(this);
		menu.add(removeFiltersSub);

		// remove filters locally
		removeFiltersLoc = new JMenuItem("Remove locally", KeyEvent.VK_T);
		removeFiltersLoc.addActionListener(this);
		menu.add(removeFiltersLoc);

		//////////////////////////////////////////////////  Algorithm        ////////////////////////
		menu = new JMenu("Algorithm");
		menu.setMnemonic(KeyEvent.VK_N);
		menuBar.add(menu);
		// select alg to run in sub 
		chooseAlgosSub = new JMenuItem("Select in Sub", KeyEvent.VK_T);
		chooseAlgosSub.addActionListener(this);
		menu.add(chooseAlgosSub);

		// select alg to run locally
		chooseAlgosLoc = new JMenuItem("Select locally", KeyEvent.VK_T);
		chooseAlgosLoc.addActionListener(this);
		menu.add(chooseAlgosLoc);

		// remove alg in sub
		removeAlgSub = new JMenuItem("Remove in Sub", KeyEvent.VK_T);
		removeAlgSub.addActionListener(this);
		menu.add(removeAlgSub);
		// remove alg locally
		removeAlgLoc = new JMenuItem("Remove locally", KeyEvent.VK_T);
		removeAlgLoc.addActionListener(this);
		menu.add(removeAlgLoc);

		//////////////////////    Camera  //////////////////////////////////////

		menu = new JMenu("Camera");
		menu.setMnemonic(KeyEvent.VK_N);
		menuBar.add(menu);

		camStartStream = new JMenuItem("Start Stream", KeyEvent.VK_T);
		camStartStream.addActionListener(this);
		menu.add(camStartStream);

		camStopStream = new JMenuItem("Stop Stream", KeyEvent.VK_T);
		camStopStream.addActionListener(this);
		menu.add(camStopStream);

		////////////////////////    Image of video    //////////////////////////
		menu = new JMenu("Files");
		menu.setMnemonic(KeyEvent.VK_N);
		menuBar.add(menu);
		// open image
		openImage = new JMenuItem("Open Image", createImageIcon("Open16.gif"));
		openImage.addActionListener(this);
		menu.add(openImage);

		// open video 
		openVideo = new JMenuItem("Open Video", createImageIcon("Open16.gif")); 
		openVideo.addActionListener(this);
		menu.add(openVideo);

		closeVideo = new JMenuItem("Close Video"); 
		closeVideo.addActionListener(this);
		menu.add(closeVideo);


		//////////////////////////////////////////////////    Record         ////////////////////////
		menu = new JMenu("Save");
		menu.setMnemonic(KeyEvent.VK_N);
		menuBar.add(menu);

		submenu2 = new JMenu("Unfiltered Video");
		submenu2.setMnemonic(KeyEvent.VK_S);

		submenu = new JMenu("Front Camera");
		submenu.setMnemonic(KeyEvent.VK_S);

		saveUFCamera = new JMenuItem("Record unfiltered video");        
		saveUFCamera.addActionListener(this);
		submenu.add(saveUFCamera);  

		stopSaveUFCamera = new JMenuItem("Stop record", KeyEvent.VK_T);
		stopSaveUFCamera.addActionListener(this);
		submenu.add(stopSaveUFCamera);
		submenu2.add(submenu);

		submenu = new JMenu("Bottom Camera");
		submenu.setMnemonic(KeyEvent.VK_S);

		saveUBCamera = new JMenuItem("Record unfiltered video", KeyEvent.VK_T);
		saveUBCamera.addActionListener(this);
		submenu.add(saveUBCamera);

		stopSaveUBCamera = new JMenuItem("Stop record", KeyEvent.VK_T);
		stopSaveUBCamera.addActionListener(this);
		submenu.add(stopSaveUBCamera);  
		submenu2.add(submenu);
		menu.add(submenu2);

		submenu2 = new JMenu("Filtered Video");
		submenu2.setMnemonic(KeyEvent.VK_S);

		submenu = new JMenu("Front Camera");
		submenu.setMnemonic(KeyEvent.VK_S);

		saveFFCamera = new JMenuItem("Record filtered video");        
		saveFFCamera.addActionListener(this);
		submenu.add(saveFFCamera);  

		stopSaveFFCamera = new JMenuItem("Stop record", KeyEvent.VK_T);
		stopSaveFFCamera.addActionListener(this);
		submenu.add(stopSaveFFCamera);
		submenu2.add(submenu);

		submenu = new JMenu("Bottom Camera");
		submenu.setMnemonic(KeyEvent.VK_S);

		saveFBCamera = new JMenuItem("Record filtered video", KeyEvent.VK_T);
		saveFBCamera.addActionListener(this);
		submenu.add(saveFBCamera);

		stopSaveFBCamera = new JMenuItem("Stop record", KeyEvent.VK_T);
		stopSaveFBCamera.addActionListener(this);
		submenu.add(stopSaveFBCamera);  
		submenu2.add(submenu);
		menu.add(submenu2);

		submenu = new JMenu("Video or Image");
		submenu.setMnemonic(KeyEvent.VK_S);

		saveFVideoOrImage = new JMenuItem("Record filtered Video or Image", KeyEvent.VK_T);
		saveFVideoOrImage.addActionListener(this);
		submenu.add(saveFVideoOrImage);

		stopSaveFVideoOrImage = new JMenuItem("Stop record", KeyEvent.VK_T);
		stopSaveFVideoOrImage.addActionListener(this);
		submenu.add(stopSaveFVideoOrImage);  
		submenu2.add(submenu);
		menu.add(submenu2);

		//////////////////////////////////////////////////  Run ordered list of filters        ////////////////////////        

		menu = new JMenu("Run");
		menu.setMnemonic(KeyEvent.VK_N);
		menuBar.add(menu);


		submenu = new JMenu("Chained");
		submenu.setMnemonic(KeyEvent.VK_S);

		runCFrontCamera = new JMenuItem("Front camera");        
		runCFrontCamera.addActionListener(this);
		submenu.add(runCFrontCamera);  

		runCBottomCamera = new JMenuItem("Bottom camera", KeyEvent.VK_T);
		runCBottomCamera.addActionListener(this);
		submenu.add(runCBottomCamera);
		menu.add(submenu);

		runCVideoImage = new JMenuItem("Video or Image", KeyEvent.VK_T);
		runCVideoImage.addActionListener(this);
		submenu.add(runCVideoImage);
		menu.add(submenu);

		submenu = new JMenu("Unchained");
		submenu.setMnemonic(KeyEvent.VK_S);

		runUCFrontCamera = new JMenuItem("Front camera");        
		runUCFrontCamera.addActionListener(this);
		submenu.add(runUCFrontCamera);  

		runUCBottomCamera = new JMenuItem("Bottom camera", KeyEvent.VK_T);
		runUCBottomCamera.addActionListener(this);
		submenu.add(runUCBottomCamera);

		runUCVideoImage = new JMenuItem("Video or Image", KeyEvent.VK_T);
		runUCVideoImage.addActionListener(this);
		submenu.add(runUCVideoImage);
		menu.add(submenu);

		////////////////////////////////////////////////// Statistic        ////////////////////////

		menu = new JMenu("Statistic");
		menu.setMnemonic(KeyEvent.VK_N);
		menuBar.add(menu);

		statisticSubServer = new JMenuItem("Sub Server Hard Disk Statistic", KeyEvent.VK_T);
		statisticSubServer.addActionListener(this);
		menu.add(statisticSubServer);

		statisticLocalServer = new JMenuItem("Local Server Hard Disk Statistic", KeyEvent.VK_T);
		statisticLocalServer.addActionListener(this);
		menu.add(statisticLocalServer);

		//////////////////////////////////////////////////  Exit        ////////////////////////

		exit = new MyMenuItem("Exit");
		exit.setMnemonic(KeyEvent.VK_N); 
		menuBar.add(exit);

		/////////////////////////////////////////////////////////////////////////////////////////////
		return menuBar;
	}

	public void actionPerformed(ActionEvent e) {

    	if(e.getSource() == subServerConnect){    		
    		
    		if(!gui.getConnection().isConnected()){
    			gui.allFiltersListSub = gui.getConnection().connect();
    			if(gui.allFiltersListSub != null){
    				try {
    					gui.allAlgorithmsListSub = readAlgos(true);
    				} catch (IOException e1) {
    				//TODO:: error
    				}
    		
    				for(int i = 0; i < gui.allAlgorithmsListSub.size(); i++){
    					if(gui.allFiltersListSub.contains(gui.allAlgorithmsListSub.get(i)))
    						gui.allFiltersListSub.remove(gui.allAlgorithmsListSub.get(i));	
    				}
    				gui.consoleListModel.addElement("Connected to sub server.");
    				enableSubServerButtons();
    				}
    			else{
    				gui.consoleListModel.addElement("Cannot connect. Check that server is up.");	
    			}
    		}
    		else{
    			gui.consoleListModel.addElement("Already Connected.");
    		}
    	}
    	//TODO:: add this to gui in linux
    	else if(e.getSource() == localServerConnect){	

    		if(!gui.getLocalConnection().isConnected()){
    			gui.allFiltersListLoc = gui.getLocalConnection().connect();
    			if(gui.allFiltersListLoc != null){
    				try {
    					gui.allAlgorithmsListLoc = readAlgos(false);
    					} catch (IOException e1) {
    						//TODO:: error
    					}
    		
    				for(int i = 0; i < gui.allAlgorithmsListSub.size(); i++){
    					if(gui.allFiltersListSub.contains(gui.allAlgorithmsListSub.get(i)))
    						gui.allFiltersListSub.remove(gui.allAlgorithmsListSub.get(i));	
    				}
    				gui.consoleListModel.addElement("Connected to local server.");
    				enableLocalServerButtons();
    			}
    			else{
    				gui.consoleListModel.addElement("Cannot connect. Check that server is up.");	
    			}
    		}
    		else{
    			gui.consoleListModel.addElement("Already Connected.");
    		}
    	}

    	else if(e.getSource() == camStartStream){
    		if(!gui.getConnection().isStreaming()){
    			gui.startStreamThread();
    			gui.consoleListModel.addElement("Start video stream.");
   			}
    		else 
   				gui.consoleListModel.addElement("Already streaming.");
    	}

		else if(e.getSource() == closeVideo){

			if(gui.getLocalConnection().streaming){
				gui.getLocalConnection().endStream();
				gui.videoStream.interrupt();
				fileOpen = -1;
				cleanFImageVideo();
				cleanUImageVideo();
			}

		}
		else if(e.getSource() == camStopStream){
			if(gui.getConnection().isStreaming()){
				gui.getConnection().endStream();
				gui.camStream.interrupt();
				gui.fCamImage = -1;
				gui.bCamImage = -1;

				cleanFFC();
				cleanUFC();
				cleanFBC();
				cleanUBC();

				gui.consoleListModel.addElement("Disconnected.");
			}
		}
		else if(e.getSource() == openVideo){
			fileOpen = 1;  	
			System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
			fc = new JFileChooser();
			int returnVal = fc.showOpenDialog(Menu.this);
			//TODO:: add filter for files
			if (returnVal == JFileChooser.APPROVE_OPTION) {
				pathToLastFile = "";   //reset path to image
				File file = fc.getSelectedFile();
				String path = fc.getSelectedFile().getPath();

				if(gui.videoStream != null){
					gui.getLocalConnection().endStream();   
					gui.videoStream.interrupt();
				}         		

				gui.initVideoThread(path); // run thread  TODO:: remove comment
				gui.consoleListModel.addElement("Video loaded from " + path);
			}
		}

		else if(e.getSource() == openImage){
			System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
			fc = new JFileChooser();
			int returnVal = fc.showOpenDialog(Menu.this);
			//TODO:: add filter for files
			if (returnVal == JFileChooser.APPROVE_OPTION) {
				File file = fc.getSelectedFile();
				String path = fc.getSelectedFile().getPath();
				Mat image = Highgui.imread(fc.getSelectedFile().getPath());
				//display unfiltered image
				gui.showImage(image, 1, -1, 0); 
				fileOpen = 0;

				if(gui.currFiltersListVideo.isEmpty()){ 
					gui.consoleListModel.addElement("In order to see filtered image you need select and run filters.");
				}
				pathToLastFile = path;
			}

		} 

		else if(e.getSource() == createSub){    		
			JTextField newFilerName = new JTextField(26);
			JPanel dialogPanel = new JPanel();
			BoxLayout layout = new BoxLayout(dialogPanel, BoxLayout.Y_AXIS);
			dialogPanel.setLayout(layout);

			JPanel row1 = new JPanel();
			FlowLayout fl = new FlowLayout();
			fl.setAlignment(FlowLayout.LEFT);
			row1.setLayout(fl);
			row1.add(new JLabel("Enter filter name:"));
			row1.add(newFilerName);
			dialogPanel.add(row1);


			row1 = new JPanel();
			row1.setLayout(fl);
			row1.add(new JLabel("Choose camera:"));

			ButtonGroup group = new ButtonGroup();
			JRadioButtonMenuItem fCam = new JRadioButtonMenuItem("Front camera");
			fCam.setSelected(true);
			fCam.setMnemonic(KeyEvent.VK_R);
			group.add(fCam);
			row1.add(fCam);

			JRadioButtonMenuItem bCam = new JRadioButtonMenuItem("Bottom camera");
			bCam.setMnemonic(KeyEvent.VK_O);
			group.add(bCam);
			row1.add(bCam);

			dialogPanel.add(row1);

			int result = JOptionPane.showConfirmDialog(null, dialogPanel, 
					"Create new filter",  JOptionPane.OK_CANCEL_OPTION,JOptionPane.PLAIN_MESSAGE);

			if (result == JOptionPane.OK_OPTION) {

				String fName = newFilerName.getText();
				if(fName!= null && fName.length() > 0){
					if(!gui.allFiltersListSub.contains(fName) && !gui.allAlgorithmsListSub.contains(fName)){
						boolean cam = fCam.isSelected();

						//cam is selected
						if ((cam && gui.currFiltersListCam1.size() > 0) || (!cam && gui.currFiltersListCam2.size() > 0)){
							if((cam && gui.fCamImage >= 0) || (!cam && gui.bCamImage >= 0)){
							try {
								if((cam && gui.fCFilterRunning == 0) || (!cam && gui.bCFilterRunning == 0)){ 
									ArrayList<String> filtersInSub = gui.getConnection().createFilter(fName, cam);
									System.out.println("Filters in server:");
									for(int i = 0; i < filtersInSub.size(); i++){
										System.out.println(filtersInSub.get(i));
									}
									if (filtersInSub != null && filtersInSub.contains(fName)){
										PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter(System.getProperty("user.dir")+"/SubAlgos.config", true)));
										out.println(fName);
										out.close();
										gui.allAlgorithmsListSub.add(fName);
										gui.consoleListModel.addElement("Algorithm " + fName + " was added.");
									}
									else{
										gui.consoleListModel.addElement("Cannot create new algorithm, there is problem in server for sub.");
									}		
								}
								else{	
									gui.consoleListModel.addElement("Cannot create new algorithm from another algorithms, only from filters.");	
								}
							} catch (IOException message) {
								gui.consoleListModel.addElement("Cannot add algorithm.");
							}
							} 
							else{
	    					  gui.consoleListModel.addElement("Run filters chained or unchained in order to create new algorithm.");
	    				  }
						}
						else{
							gui.consoleListModel.addElement("Select at least one filter in order to create new filter.");
						}
					}
					else
						gui.consoleListModel.addElement("Cannot create new algorithm because there already exist filter or algorithm with the same name: " + fName + ".");

				}
			}		
		}

		else if(e.getSource() == createLoc){    		
			JTextField newFilerName = new JTextField(26);
			JPanel dialogPanel = new JPanel();
			BoxLayout layout = new BoxLayout(dialogPanel, BoxLayout.Y_AXIS);
			dialogPanel.setLayout(layout);

			JPanel row1 = new JPanel();
			FlowLayout fl = new FlowLayout();
			fl.setAlignment(FlowLayout.LEFT);
			row1.setLayout(fl);
			row1.add(new JLabel("Enter filter name:"));
			row1.add(newFilerName);
			dialogPanel.add(row1);


			int result = JOptionPane.showConfirmDialog(null, dialogPanel, 
					"Create new filter",  JOptionPane.OK_CANCEL_OPTION,JOptionPane.PLAIN_MESSAGE);

			if (result == JOptionPane.OK_OPTION) {

				String fName = newFilerName.getText();
				if(fName!= null && fName.length() > 0){
					if(!gui.allFiltersListLoc.contains(fName) && !gui.allAlgorithmsListLoc.contains(fName)){
						if(gui.VideoImage >= 0){
						if(gui.currFiltersListVideo.size() > 0){
							try {
								if(gui.videoFilterRunning == 0){

									ArrayList<String> localfilters = gui.getLocalConnection().createFilter(fName); 	
									if (localfilters != null && localfilters.contains(fName)){
										PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter(System.getProperty("user.dir")+"/LocAlgos.config", true)));
										out.println(fName);
										out.close();  

										gui.allAlgorithmsListLoc.add(fName);
										gui.consoleListModel.addElement("Algorithm " + fName + " was added.");
									}
									else{
										gui.consoleListModel.addElement("Cannot create new algorithm, there is problem in server for sub.)");
									}  					    
								}
								else{	
									gui.consoleListModel.addElement("Cannot create new algorithm from another algorithms (only from filters).");	
								}
							} catch (IOException message) {
								gui.consoleListModel.addElement("Cannot add algorithm.");
							}
  	    				}
  	    				else{
  	    					gui.consoleListModel.addElement("Run filters chained or unchained in order to create new algorithm.");
  	    				}							
						}   
						else{
							gui.consoleListModel.addElement("Select at least one filter in order to create new filter.");
						}
					}
					else
						gui.consoleListModel.addElement("Cannot create new algorithm because there already exist filter or algorithm with the same name: " + fName + ".");	    	  
				}
			}		
		}

		else if(e.getSource() == removeFiltersSub){
			if(gui.allFiltersListSub.size() > 0){

				String [] possibilities = new String [gui.allFiltersListSub.size()];

				for(int i = 0; i < gui.allFiltersListSub.size(); i++)
					possibilities[i] = gui.allFiltersListSub.get(i);

				String filterName = (String)JOptionPane.showInputDialog(
						null,
						"Select filter to remove:",
						"Remove Filter",
						JOptionPane.PLAIN_MESSAGE,
						// icon,
						null,
						possibilities,
						possibilities[0]);

				//If a string was returned, say so.
				if ((filterName != null) && (filterName.length() > 0)) {
					if(!(gui.currFiltersListCam1.contains(filterName) ||
							gui.currFiltersListCam2.contains(filterName))){

						ArrayList<String> filtersisServer = gui.getConnection().removeFilter(filterName);
						if(filtersisServer != null && (!filtersisServer.contains(filterName))){
							gui.allFiltersListSub.remove(filterName);
							gui.consoleListModel.addElement("Filter " + filterName + " removed.");
						}
						else{
							gui.consoleListModel.addElement("Cannot remove filter. Check that it is not a built in filter. If so, there is a problem in sub server.");
						}		
					}
					else{
						gui.consoleListModel.addElement("Cannot remove filter because it is currently in use.");
					}
				}
			}
			else{
				JOptionPane.showMessageDialog(null, "There are no filters.");
			}
		}

		else if(e.getSource() == removeFiltersLoc){
			if(gui.allFiltersListLoc.size() > 0){

				String [] possibilities = new String [gui.allFiltersListLoc.size()]; 

				for(int i = 0; i < gui.allFiltersListLoc.size(); i++)
					possibilities[i] = gui.allFiltersListLoc.get(i);

				String filterName = (String)JOptionPane.showInputDialog(
						null,
						"Select filter to remove:",
						"Remove Filter",
						JOptionPane.PLAIN_MESSAGE,
						// icon,
						null,
						possibilities,
						possibilities[0]);

				//If a string was returned, say so.
				if((filterName != null) && (filterName.length() > 0)) {
					if(!gui.currFiltersListVideo.contains(filterName)){
						ArrayList<String> localfilters = gui.getLocalConnection().removeFilter(filterName);
						if(localfilters != null && (!localfilters.contains(filterName))){
							gui.allFiltersListLoc.remove(filterName);
							gui.consoleListModel.addElement("Filter " + filterName + " removed.");
						}
						else{
							gui.consoleListModel.addElement("Cannot remove filter. Check that it is not a built in filter. If so, there is a problem in sub server.");
						}		
					}
					else{
						gui.consoleListModel.addElement("Cannot remove filter because it is currently in use.");
					}
				}
			}
			else{
				JOptionPane.showMessageDialog(null, "There are no filters.");
			}
		}

		else if(e.getSource() == removeAlgSub){
			if(gui.allAlgorithmsListSub.size() > 0){
				String [] possibilities = new String [gui.allAlgorithmsListSub.size()];

				for(int i = 0; i < gui.allAlgorithmsListSub.size(); i++)
					possibilities[i] = gui.allAlgorithmsListSub.get(i);

				String filterName = (String)JOptionPane.showInputDialog(
						null,
						"Select algorithm to remove:",
						"Remove Algorithm",
						JOptionPane.PLAIN_MESSAGE,
						// icon,
						null,
						possibilities,
						possibilities[0]);

				//If a string was returned, say so.
				if ((filterName != null) && (filterName.length() > 0)) {
					if(!(gui.currFiltersListCam1.contains(filterName) ||
							gui.currFiltersListCam2.contains(filterName) ||
							gui.currFiltersListVideo.contains(filterName))){
						try {
							ArrayList<String> filtersInSub = gui.getConnection().removeFilter(filterName);
							if(filtersInSub != null && (!filtersInSub.contains(filterName))){
								PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter(System.getProperty("user.dir")+"/SubAlgos.config", false)));
								gui.allAlgorithmsListSub.remove(filterName);
								for(int i = 0; i < gui.allAlgorithmsListSub.size(); i++){
									out.println(gui.allAlgorithmsListSub.get(i));   
								}
								out.close();
								gui.consoleListModel.addElement("Algorithm " + filterName + " was removed.");  			    		  
							}
							else{
								gui.consoleListModel.addElement("Cannot remove algorithm. There is a problem is server for sub.");  
							}  
						} catch (IOException message) {
							gui.consoleListModel.addElement("Cannot remove algorithm.");
						}  				  
					}
					else{
						gui.consoleListModel.addElement("Cannot remove algorithm because it is currently in use.");
					}
				}
			}
			else{
				JOptionPane.showMessageDialog(null, "There are no algorithms.");
			}
		}
		else if(e.getSource() == removeAlgLoc){
			if(gui.allAlgorithmsListLoc.size() > 0){
				String [] possibilities = new String [gui.allAlgorithmsListLoc.size()];

				for(int i = 0; i < gui.allAlgorithmsListLoc.size(); i++)
					possibilities[i] = gui.allAlgorithmsListLoc.get(i);

				String filterName = (String)JOptionPane.showInputDialog(
						null,
						"Select algorithm to remove:",
						"Remove Algorithm",
						JOptionPane.PLAIN_MESSAGE,
						// icon,
						null,
						possibilities,
						possibilities[0]);

				//If a string was returned, say so.
				if ((filterName != null) && (filterName.length() > 0)) {
					if(!(gui.currFiltersListVideo.contains(filterName))){
						try {
							ArrayList<String> localfilters = gui.getLocalConnection().removeFilter(filterName);
							if(localfilters != null && (!localfilters.contains(filterName))){
								PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter(System.getProperty("user.dir")+"/LocAlgos.config", false)));
								gui.allAlgorithmsListLoc.remove(filterName);
								for(int i = 0; i < gui.allAlgorithmsListLoc.size(); i++){
									out.println(gui.allAlgorithmsListLoc.get(i));   
								}
								out.close();
								gui.consoleListModel.addElement("Algorithm " + filterName + " was removed.");  			    		  
							}
							else{
								gui.consoleListModel.addElement("Cannot remove algorithm. There is a problem is server for sub.");  
							}  
						} catch (IOException message) {
							gui.consoleListModel.addElement("Cannot remove algorithm.");
						}  				  
					}
					else{
						gui.consoleListModel.addElement("Cannot remove algorithm because it is currently in use.");
					}
				}
			}
			else{
				JOptionPane.showMessageDialog(null, "There are no algorithms.");
			}
		}
		else if(e.getSource() == chooseAlgosSub){

			if(gui.allAlgorithmsListSub.size() > 0){

				JPanel dialogPanel = new JPanel();
				BoxLayout layout = new BoxLayout(dialogPanel, BoxLayout.Y_AXIS);
				dialogPanel.setLayout(layout);

				JPanel row1 = new JPanel();
				FlowLayout fl = new FlowLayout();
				fl.setAlignment(FlowLayout.LEFT);
				row1.setLayout(fl);
				row1.add(new JLabel("Choose:"));
				ButtonGroup group = new ButtonGroup();
				JRadioButtonMenuItem fCam = new JRadioButtonMenuItem("Front camera");
				fCam.setSelected(true);
				fCam.setMnemonic(KeyEvent.VK_R);
				group.add(fCam);
				row1.add(fCam);
				JRadioButtonMenuItem bCam = new JRadioButtonMenuItem("Bottom camera");
				bCam.setMnemonic(KeyEvent.VK_O);
				group.add(bCam);
				row1.add(bCam);
				dialogPanel.add(row1);


				////////////////////////////////////////////////////////////////////////////////
				//panel for existing filters
				JPanel row2 = new JPanel();
				row2.setLayout(new FlowLayout());   	     

				//add filters to LListModel
				lListModel = new DefaultListModel<String>();
				lList = new JList<String>(lListModel);
				JPanel lFilterList = new JPanel();  

				JScrollPane lListScroller = new JScrollPane(lList);
				lListScroller.setPreferredSize(new Dimension(200, 240));
				lFilterList.setBounds(20, 25, 200, 245);
				lFilterList.add(lListScroller);
				TitledBorder title = new TitledBorder(new EtchedBorder(), "Current Algorithms:",TitledBorder.LEFT, TitledBorder.LEFT);
				lListScroller.setBorder(title);
				row2.add(lFilterList);

				//add filters to current filters list
				for(int i = 0; i < gui.allAlgorithmsListSub.size(); i++)
					lListModel.addElement(gui.allAlgorithmsListSub.get(i));

				// arrows

				removeButton = new BasicArrowButton(BasicArrowButton.WEST);
				removeButton.setBounds(285 , 415, 20, 20);
				removeButton.addActionListener(this);
				row2.add(removeButton);

				addButton = new BasicArrowButton(BasicArrowButton.EAST);
				addButton.setBounds(385 , 415, 20, 20);
				addButton.addActionListener(this);
				row2.add(addButton);

				//panel for chosen filters 	      
				rListModel = new DefaultListModel<String>();
				rList = new JList<String>(rListModel);
				JPanel RFilterList = new JPanel();

				JScrollPane RListScroller = new JScrollPane(rList);
				RListScroller.setPreferredSize(new Dimension(200, 240));
				RFilterList.setBounds(20, 25, 200, 245);
				RFilterList.add(RListScroller);

				TitledBorder rTitle = new TitledBorder(new EtchedBorder(), "Chosen Algorithms:",TitledBorder.LEFT, TitledBorder.LEFT);
				RListScroller.setBorder(rTitle);

				row2.add(RFilterList);      
				dialogPanel.add(row2);


				int result = JOptionPane.showConfirmDialog(null, dialogPanel, 
						"Choose Algorithms To Run",  JOptionPane.OK_CANCEL_OPTION,JOptionPane.PLAIN_MESSAGE);

				if (result == JOptionPane.OK_OPTION) {
					if(rListModel.size() > 0){
						if(fCam.isSelected()){
							gui.currFiltersListCam1.clear();
							gui.frontCamList.removeAllChildren();
							//add filters to current filters list in gui
							for(int i = 0; i < rListModel.size(); i++){
								gui.currFiltersListCam1.add(rListModel.get(i));
								gui.frontCamList.add(new DefaultMutableTreeNode(rListModel.get(i)));
							} 
							gui.fCamImage = -1;
							gui.fCFilterRunning = 1;
							gui.model.reload(gui.root);  
							this.cleanUFC();
						}  	
						else if(bCam.isSelected()){
							gui.currFiltersListCam2.clear();
							gui.bottomCamList.removeAllChildren();
							//add filters to current filters list in gui
							for(int i = 0; i < rListModel.size(); i++){
								gui.currFiltersListCam2.add(rListModel.get(i));
								gui.bottomCamList.add(new DefaultMutableTreeNode(rListModel.get(i)));
							}
							gui.bCamImage = -1;
							gui.bCFilterRunning = 1;
							gui.model.reload(gui.root); 
							this.cleanUBC();
						}
					}
				}
			}
			else{
				JOptionPane.showMessageDialog(null, "There are no algorithms.");
			}
		}

		else if(e.getSource() == chooseAlgosLoc){

			if(gui.allAlgorithmsListLoc.size() > 0){

				JPanel dialogPanel = new JPanel();
				BoxLayout layout = new BoxLayout(dialogPanel, BoxLayout.Y_AXIS);
				dialogPanel.setLayout(layout);

				////////////////////////////////////////////////////////////////////////////////
				//panel for existing filters
				JPanel row2 = new JPanel();
				row2.setLayout(new FlowLayout());   	     

				//add filters to LListModel
				lListModel = new DefaultListModel<String>();
				lList = new JList<String>(lListModel);
				JPanel lFilterList = new JPanel();  

				JScrollPane lListScroller = new JScrollPane(lList);
				lListScroller.setPreferredSize(new Dimension(200, 240));
				lFilterList.setBounds(20, 25, 200, 245);
				lFilterList.add(lListScroller);
				TitledBorder title = new TitledBorder(new EtchedBorder(), "Current Algorithms:",TitledBorder.LEFT, TitledBorder.LEFT);
				lListScroller.setBorder(title);
				row2.add(lFilterList);

				//add filters to current filters list
				for(int i = 0; i < gui.allAlgorithmsListLoc.size(); i++)
					lListModel.addElement(gui.allAlgorithmsListLoc.get(i));

				// arrows

				removeButton = new BasicArrowButton(BasicArrowButton.WEST);
				removeButton.setBounds(285 , 415, 20, 20);
				removeButton.addActionListener(this);
				row2.add(removeButton);

				addButton = new BasicArrowButton(BasicArrowButton.EAST);
				addButton.setBounds(385 , 415, 20, 20);
				addButton.addActionListener(this);
				row2.add(addButton);

				//panel for chosen filters 	      
				rListModel = new DefaultListModel<String>();
				rList = new JList<String>(rListModel);
				JPanel RFilterList = new JPanel();

				JScrollPane RListScroller = new JScrollPane(rList);
				RListScroller.setPreferredSize(new Dimension(200, 240));
				RFilterList.setBounds(20, 25, 200, 245);
				RFilterList.add(RListScroller);

				TitledBorder rTitle = new TitledBorder(new EtchedBorder(), "Chosen Algorithms:",TitledBorder.LEFT, TitledBorder.LEFT);
				RListScroller.setBorder(rTitle);

				row2.add(RFilterList);      
				dialogPanel.add(row2);


				int result = JOptionPane.showConfirmDialog(null, dialogPanel, 
						"Choose Algorithms To Run",  JOptionPane.OK_CANCEL_OPTION,JOptionPane.PLAIN_MESSAGE);

				if (result == JOptionPane.OK_OPTION) {
					if(rListModel.size() > 0){
						gui.currFiltersListVideo.clear();
						gui.videoList.removeAllChildren();
						//add filters to current filters list in gui
						for(int i = 0; i < rListModel.size(); i++){
							gui.currFiltersListVideo.add(rListModel.get(i));
							gui.videoList.add(new DefaultMutableTreeNode(rListModel.get(i)));
						}
						gui.VideoImage = -1;
						gui.videoFilterRunning = 1;
						gui.model.reload(gui.root); 	
						this.cleanFImageVideo();
					}
				}
			}
			else{
				JOptionPane.showMessageDialog(null, "There are no algorithms.");
			}
		}
		else if(e.getSource() == saveFVideoOrImage){
			if(videoImg == 0){ 
				gui.getLocalConnection().startRecording(gui.currFiltersListVideo);
				gui.filteredVideoStream.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.red), null), "Video or Image: Filtered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
				videoImg = 1;
				gui.consoleListModel.addElement("Start recording.");
			}
			else{
				gui.consoleListModel.addElement("Already recording.");
			}   	    	
		} 	
		else if(e.getSource() == stopSaveFVideoOrImage){
			if(videoImg == 1){ 
				gui.getLocalConnection().stopRecording(gui.currFiltersListVideo);
				gui.filteredVideoStream.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.gray), null), "Video or Image: Filtered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
				videoImg = 0;
				gui.consoleListModel.addElement("Stop recording.");
			}  	    	
		}
		else if(e.getSource() == saveFFCamera){
			if(fFCam == 0){ 
				gui.getConnection().startRecording(gui.currFiltersListCam1,  true);
				gui.filteredStreamCam1.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.red), null), "Front Camera: Filtered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
				fFCam = 1;
				gui.consoleListModel.addElement("Start recording.");
			}
			else{
				gui.consoleListModel.addElement("Already recording.");
			}
		}
		else if(e.getSource() == saveFBCamera){  		
			if(fBCam == 0){ 
				gui.getConnection().startRecording(gui.currFiltersListCam2, false);
				gui.filteredStreamCam2.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.red), null), "Bottom Camera: Filtered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
				fBCam = 1;
				gui.consoleListModel.addElement("Start recording.");
			}
			else{
				gui.consoleListModel.addElement("Already recording.");
			}
		}
		else if(e.getSource() == stopSaveFFCamera){
			if(fFCam == 1){ 
				gui.getConnection().stopRecording(gui.currFiltersListCam1, true);
				gui.filteredStreamCam1.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.gray), null), "Front Camera: Filtered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
				fFCam = 0; 
				gui.consoleListModel.addElement("Stop recording.");
			}
		}
		else if(e.getSource() == stopSaveFBCamera){
			if(fBCam == 1){	
				gui.getConnection().stopRecording(gui.currFiltersListCam2, false);
				gui.filteredStreamCam2.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.gray), null), "Bottom Camera: Filtered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
				fBCam = 0;
				gui.consoleListModel.addElement("Stop recording.");
			}
		}

		else if(e.getSource() == statisticSubServer){
			String stats = gui.getConnection().getHDDStats();
			if(stats != null){
				gui.consoleListModel.addElement("Hard disk statistic in sub server:");  
				String[] str = stats.split("\n");
				for(int i = 0; i < str.length; i++)
					gui.consoleListModel.addElement(str[i]);
			}    	
		}
		else if(e.getSource() == statisticLocalServer){
			String stats = gui.getLocalConnection().getHDDStats();
			if(stats != null){
				gui.consoleListModel.addElement("Hard disk statistic in local server:");
				String[] str = stats.split("\n");
				for(int i = 0; i < str.length; i++)
					gui.consoleListModel.addElement(str[i]);
			}    	
		}

		else if(e.getSource() == saveUFCamera){
			if(uFCam == 0){  
				gui.getConnection().startRecordingUnfiltered(true);
				gui.unfilteredStreamCam1.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.red), null), "Front Camera: Unfiltered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
				uFCam = 1;
				gui.consoleListModel.addElement("Start recording.");
			}
			else{
				gui.consoleListModel.addElement("Already recording.");	
			}
		}
		else if(e.getSource() == saveUBCamera){
			if(uBCam == 0){ 
				gui.getConnection().startRecordingUnfiltered(false);
				gui.unfilteredStreamCam2.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.red), null), "Bottom Camera: Unfiltered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
				uBCam = 1;
				gui.consoleListModel.addElement("Start recording.");
			}
			else{
				gui.consoleListModel.addElement("Already recording.");
			}
		}
		else if(e.getSource() == stopSaveUFCamera){
			if(uFCam == 1){
				gui.getConnection().stopRecordingUnfiltered(true);
				gui.unfilteredStreamCam1.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.gray), null), "Front Camera: Unfiltered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
				uFCam = 0;
				gui.consoleListModel.addElement("Stop recording.");
			}
		}
		else if(e.getSource() == stopSaveUBCamera){
			if(uBCam == 1){ 
				gui.getConnection().stopRecordingUnfiltered(false);
				gui.unfilteredStreamCam2.setBorder(new TitledBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(Color.gray), null), "Bottom Camera: Unfiltered Stream", TitledBorder.CENTER, TitledBorder.CENTER));
				uBCam = 0;
				gui.consoleListModel.addElement("Stop recording.");
			}
		}

		else if(e.getSource() == runCVideoImage){ 

			if(!gui.currFiltersListVideo.isEmpty()){
				boolean res = gui.getLocalConnection().chainedFiltersList(gui.currFiltersListVideo);

				if(this.fileOpen == 0 && !res){ 
					//display filtered image  
					System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
					if(pathToLastFile.length() > 0){
						gui.filteredImages = gui.getLocalConnection().filterImage(pathToLastFile);
					
						if(gui.filteredImages != null ){
							gui.VideoImage = 0; //default filter index
							gui.showImage(gui.filteredImages.get(0), 1, -1, 1);
						}
					}
				
				}
				if(this.fileOpen == 1 && !res)
					gui.VideoImage = 0; //default filter index
			}
			else{
				gui.consoleListModel.addElement("Select filters to run.");
			}
		}

		else if(e.getSource() == runUCVideoImage){
			//TODO:: check if video or image is on 
			if(!gui.currFiltersListVideo.isEmpty()){
				boolean res = gui.getLocalConnection().unorderedFiltersList(gui.currFiltersListVideo);
				if(this.fileOpen == 0 && !res){
					
					if(pathToLastFile.length() > 0){
						gui.filteredImages = gui.getLocalConnection().filterImage(pathToLastFile);
					
						if(gui.filteredImages != null ){
							gui.VideoImage = 0; //default filter index
							gui.showImage(gui.filteredImages.get(0), 1, -1, 1);
						}
					}
	
				}
				if(this.fileOpen == 1 && !res)
					gui.VideoImage = 0; //default filter index
			}
			else{
				gui.consoleListModel.addElement("Select filters to run.");
			}
		}

		else if(e.getSource() == runCFrontCamera){
			if(!gui.currFiltersListCam1.isEmpty()){
				if(!gui.getConnection().chainedFiltersList(gui.currFiltersListCam1, true));
				gui.fCamImage = 0;     			//default filter index

				//TODO:: remove later
				for(int i =0; i< gui.currFiltersListCam1.size();i++)
					System.out.println(gui.currFiltersListCam1.get(i));
				System.out.println();
			}
			else{
				gui.consoleListModel.addElement("Select filters to run.");
			}
		}
		else if(e.getSource() == runCBottomCamera){
			if(!gui.currFiltersListCam2.isEmpty()){
				if(!gui.getConnection().chainedFiltersList(gui.currFiltersListCam2, false));
				gui.bCamImage = 0;     			//default filter index
			}
			else{
				gui.consoleListModel.addElement("Select filters to run.");
			}
		}
		else if(e.getSource() == runUCFrontCamera){
			if(!gui.currFiltersListCam1.isEmpty()){
				if(!gui.getConnection().unorderedFiltersList(gui.currFiltersListCam1, true));
				gui.fCamImage = 0; //default filter index
			}
			else{
				gui.consoleListModel.addElement("Select filters to run.");
			}
		}
		else if(e.getSource() == runUCBottomCamera){
			if(!gui.currFiltersListCam2.isEmpty()){
				if(!gui.getConnection().unorderedFiltersList(gui.currFiltersListCam2, false));			
				gui.bCamImage = 0;//default filter index
			}
			else{
				gui.consoleListModel.addElement("Select filters to run.");
			}
		}

		else if(e.getSource() == chooseFiltersSub){

			JPanel dialogPanel = new JPanel();
			BoxLayout layout = new BoxLayout(dialogPanel, BoxLayout.Y_AXIS);
			dialogPanel.setLayout(layout);

			JPanel row1 = new JPanel();
			FlowLayout fl = new FlowLayout();
			fl.setAlignment(FlowLayout.LEFT);
			row1.setLayout(fl);
			row1.add(new JLabel("Choose:"));
			ButtonGroup group = new ButtonGroup();
			JRadioButtonMenuItem fCam = new JRadioButtonMenuItem("Front camera");
			fCam.setSelected(true);
			fCam.setMnemonic(KeyEvent.VK_R);
			group.add(fCam);
			row1.add(fCam);
			JRadioButtonMenuItem bCam = new JRadioButtonMenuItem("Bottom camera");
			bCam.setMnemonic(KeyEvent.VK_O);
			group.add(bCam);
			row1.add(bCam);
			dialogPanel.add(row1);

			////////////////////////////////////////////////
			//panel for existing filters
			JPanel row2 = new JPanel();
			row2.setLayout(new FlowLayout());   	     

			//add filters to LListModel
			lListModel = new DefaultListModel<String>();
			lList = new JList<String>(lListModel);
			JPanel lFilterList = new JPanel();  

			JScrollPane lListScroller = new JScrollPane(lList);
			lListScroller.setPreferredSize(new Dimension(200, 240));
			lFilterList.setBounds(20, 25, 200, 245);
			lFilterList.add(lListScroller);
			TitledBorder title = new TitledBorder(new EtchedBorder(), "Current Filters:",TitledBorder.LEFT, TitledBorder.LEFT);
			lListScroller.setBorder(title);
			row2.add(lFilterList);

			//add filters to current filters list
			for(int i = 0; i < gui.allFiltersListSub.size(); i++)
				lListModel.addElement(gui.allFiltersListSub.get(i));

			// arrows

			removeButton = new BasicArrowButton(BasicArrowButton.WEST);
			removeButton.setBounds(285 , 415, 20, 20);
			removeButton.addActionListener(this);
			row2.add(removeButton);

			addButton = new BasicArrowButton(BasicArrowButton.EAST);
			addButton.setBounds(385 , 415, 20, 20);
			addButton.addActionListener(this);
			row2.add(addButton);

			//panel for chosen filters 	      
			rListModel = new DefaultListModel<String>();
			rList = new JList<String>(rListModel);
			JPanel RFilterList = new JPanel();

			JScrollPane RListScroller = new JScrollPane(rList);
			RListScroller.setPreferredSize(new Dimension(200, 240));
			RFilterList.setBounds(20, 25, 200, 245);
			RFilterList.add(RListScroller);

			TitledBorder rTitle = new TitledBorder(new EtchedBorder(), "Chosen Filters:",TitledBorder.LEFT, TitledBorder.LEFT);
			RListScroller.setBorder(rTitle);

			row2.add(RFilterList);      
			dialogPanel.add(row2);


			int result = JOptionPane.showConfirmDialog(null, dialogPanel, 
					"Choose Filters To Run",  JOptionPane.OK_CANCEL_OPTION,JOptionPane.PLAIN_MESSAGE);

			if (result == JOptionPane.OK_OPTION) {
				if(rListModel.size() > 0){
					if(fCam.isSelected()){
						gui.currFiltersListCam1.clear();
						gui.frontCamList.removeAllChildren();
						//add filters to current filters list in gui
						for(int i = 0; i < rListModel.size(); i++){
							gui.currFiltersListCam1.add(rListModel.get(i));
							gui.frontCamList.add(new DefaultMutableTreeNode(rListModel.get(i)));
						} 
						gui.fCamImage = -1;
						gui.fCFilterRunning = 0;
						gui.model.reload(gui.root);
						cleanFFC();
					}

					else if(bCam.isSelected()){
						gui.currFiltersListCam2.clear();
						gui.bottomCamList.removeAllChildren();
						//add filters to current filters list in gui
						for(int i = 0; i < rListModel.size(); i++){
							gui.currFiltersListCam2.add(rListModel.get(i));
							gui.bottomCamList.add(new DefaultMutableTreeNode(rListModel.get(i)));
						}
						gui.bCamImage = -1;
						gui.bCFilterRunning = 0;
						gui.model.reload(gui.root);
						cleanFBC();
					} 
				}
			}

		}

		else if(e.getSource() == chooseFiltersLoc){

			JPanel dialogPanel = new JPanel();
			BoxLayout layout = new BoxLayout(dialogPanel, BoxLayout.Y_AXIS);
			dialogPanel.setLayout(layout);

			////////////////////////////////////////////////
			//panel for existing filters
			JPanel row2 = new JPanel();
			row2.setLayout(new FlowLayout());   	     

			//add filters to LListModel
			lListModel = new DefaultListModel<String>();
			lList = new JList<String>(lListModel);
			JPanel lFilterList = new JPanel();  

			JScrollPane lListScroller = new JScrollPane(lList);
			lListScroller.setPreferredSize(new Dimension(200, 240));
			lFilterList.setBounds(20, 25, 200, 245);
			lFilterList.add(lListScroller);
			TitledBorder title = new TitledBorder(new EtchedBorder(), "Current Filters:",TitledBorder.LEFT, TitledBorder.LEFT);
			lListScroller.setBorder(title);
			row2.add(lFilterList);

			//add filters to current filters list
			for(int i = 0; i < gui.allFiltersListLoc.size(); i++)
				lListModel.addElement(gui.allFiltersListLoc.get(i));

			// arrows

			removeButton = new BasicArrowButton(BasicArrowButton.WEST);
			removeButton.setBounds(285 , 415, 20, 20);
			removeButton.addActionListener(this);
			row2.add(removeButton);

			addButton = new BasicArrowButton(BasicArrowButton.EAST);
			addButton.setBounds(385 , 415, 20, 20);
			addButton.addActionListener(this);
			row2.add(addButton);

			//panel for chosen filters 	      
			rListModel = new DefaultListModel<String>();
			rList = new JList<String>(rListModel);
			JPanel RFilterList = new JPanel();

			JScrollPane RListScroller = new JScrollPane(rList);
			RListScroller.setPreferredSize(new Dimension(200, 240));
			RFilterList.setBounds(20, 25, 200, 245);
			RFilterList.add(RListScroller);

			TitledBorder rTitle = new TitledBorder(new EtchedBorder(), "Chosen Filters:",TitledBorder.LEFT, TitledBorder.LEFT);
			RListScroller.setBorder(rTitle);

			row2.add(RFilterList);      
			dialogPanel.add(row2);


			int result = JOptionPane.showConfirmDialog(null, dialogPanel, 
					"Choose Filters To Run",  JOptionPane.OK_CANCEL_OPTION,JOptionPane.PLAIN_MESSAGE);

			if (result == JOptionPane.OK_OPTION) {
				if(rListModel.size() > 0){
					gui.currFiltersListVideo.clear();
					gui.videoList.removeAllChildren();
					//add filters to current filters list in gui
					for(int i = 0; i < rListModel.size(); i++){
						gui.currFiltersListVideo.add(rListModel.get(i));
						gui.videoList.add(new DefaultMutableTreeNode(rListModel.get(i)));
					}
					gui.VideoImage = -1;
					gui.videoFilterRunning = 0;
					gui.model.reload(gui.root); 
					this.cleanFImageVideo();
				}
			}	
		}

		else if(e.getSource() == addButton){
			String selectedValue = lList.getSelectedValue(); 
			if(selectedValue != null){
				if(rListModel.contains(selectedValue))
					gui.consoleListModel.addElement("You already chose this filter.");
				else
					rListModel.addElement(selectedValue);
			}
			else
				gui.consoleListModel.addElement("You have not selected a filter.");
		}
		else if(e.getSource() == removeButton){
			String selectedValue = rList.getSelectedValue(); 
			if(selectedValue != null)
				rListModel.removeElement(selectedValue);
			else
				gui.consoleListModel.addElement("You have not selected a filter.");
		}
		else if(e.getSource() == openFilterButton){
			fc = new JFileChooser();
			FileNameExtensionFilter filter = new FileNameExtensionFilter(
					"so Images", "so");
			fc.setFileFilter(filter);
			if (fc.showOpenDialog(Menu.this) == JFileChooser.APPROVE_OPTION){
				filterPath = fc.getSelectedFile().getPath();
				filterW.setText(filterPath);
			}

			else {
				gui.consoleListModel.addElement("Cann't open filter file.");
			}

		}
		else if(e.getSource() == openConfigButton){
			fc = new JFileChooser();
			FileNameExtensionFilter filter = new FileNameExtensionFilter("config Images", "config");
			fc.setFileFilter(filter);
			if (fc.showOpenDialog(Menu.this) == JFileChooser.APPROVE_OPTION){
				configPath = fc.getSelectedFile().getPath();
				this.configW.setText(configPath);
			}
		}

		else if(e.getSource() == addSub)
		{
			int result = createAddDialog();

			if (result == JOptionPane.OK_OPTION) {

				this.filterPath = this.filterW.getText();
				// this.filterPath = "/home/eliranko/workspace/StupidClient/a.so";
				this.configPath = this.configW.getText();
				// this.configPath = "/home/eliranko/workspace/StupidClient/a.config";
				if(filterPath.length() > 0 && configPath.length() >0){
					int index = filterPath.lastIndexOf( '/' );
					String fileName = filterPath.substring(index+1, filterPath.length() - 3);
					//  String fileName = "a";
					index = configPath.lastIndexOf( '/' );

					//  if(fileName.equals(configName)){

					if(!gui.allFiltersListSub.contains(fileName) && !gui.allAlgorithmsListSub.contains(fileName)){

						File newFilter = new File(this.filterPath);
						File config = new File(this.configPath);

						String newPath = System.getProperty("user.dir")+"/" +  fileName + ".config";
						try {
							copyFile(configPath, newPath);
						} catch (IOException e1) {
							gui.consoleListModel.addElement("Cannot copy configuration file.");
							filterPath = "";
							configPath = "";
							return;
						}  	    				  

						if(newFilter.isFile() && config.isFile()){			  
							ArrayList<String> newFilterList1 = gui.getConnection().addFilter(filterPath, configPath);
							ArrayList<String> a = newFilterList1;
							for(int i = 0 ; i< a.size(); i++)
								System.out.println(a.get(i));
							System.out.println("asadsaDASDSADSAD\t "+fileName);
							if(newFilterList1 != null && newFilterList1.contains(fileName)){
								gui.consoleListModel.addElement("Filter " + fileName + " added to server");
								gui.allFiltersListSub.add(fileName);
							}
							else{
								gui.consoleListModel.addElement("Cann't add a new filter. Check files or duplication in files.");
							}
						}
						else{
							gui.consoleListModel.addElement("Path to the files is incorrect."); 
						}
					}
					else {
						gui.consoleListModel.addElement("Already exist filter or algorithm with the same name: " + fileName + ".");
					}
				}
				//reset path to the files
				filterPath = "";
				configPath = "";
			}

			if (result == JOptionPane.NO_OPTION) {
				filterPath = "";
				configPath = "";
			}

		}
		else if(e.getSource() == addLoc)
		{
			int result = createAddDialog();	      

			if (result == JOptionPane.OK_OPTION) {

				this.filterPath = this.filterW.getText();
				this.configPath = this.configW.getText();
				if(filterPath.length() > 0 && configPath.length() >0){
					int index = filterPath.lastIndexOf( '/' );
					String fileName = filterPath.substring(index+1, filterPath.length() - 3);
					index = configPath.lastIndexOf( '/' );

					//  if(fileName.equals(configName)){

					if(!gui.allFiltersListLoc.contains(fileName) && !gui.allAlgorithmsListLoc.contains(fileName)){

						File newFilter = new File(this.filterPath);
						File config = new File(this.configPath);

						String newPath = System.getProperty("user.dir")+"/" +  fileName + ".config";
						try {
							copyFile(configPath, newPath);
						} catch (IOException e1) {
							gui.consoleListModel.addElement("Cannot copy configuration file");
							filterPath = "";
							configPath = "";
							return;
						}  	    				  

						if(newFilter.isFile() && config.isFile()){			  
							ArrayList<String> localFilterList = gui.getLocalConnection().addFilter(filterPath, configPath);
							if(localFilterList != null && localFilterList.contains(fileName)){
								gui.consoleListModel.addElement("Filter " + fileName + " added to local server");
								gui.allFiltersListLoc.add(fileName);
							}
							else{
								gui.consoleListModel.addElement("Cann't add a new filter. Check files or duplication in files.");
							}
						}
						else{
							gui.consoleListModel.addElement("Path to the files is incorrect."); 
						}
					}
					else {
						gui.consoleListModel.addElement("Already exist filter or algorithm with the same name: " + fileName + ".");
					}
				}
				//reset path to the files
				filterPath = "";
				configPath = "";
			}

			if (result == JOptionPane.NO_OPTION) {
				filterPath = "";
				configPath = "";
			}

		}
		else{

			System.out.println(e.getActionCommand());
		}
	}


	// Returns an ImageIcon, or null if the path was invalid.
	protected static ImageIcon createImageIcon(String path) {
		java.net.URL imgURL = Menu.class.getResource(path);
		if (imgURL != null) {
			return new ImageIcon(imgURL);
		} else {
			System.err.println("Couldn't find file: " + path);
			return null;
		}
	}

	public void setGui(ClientGUI gui){
		this.gui = gui;
	}


	//class for exit and run
	private class MyMenuItem extends JMenuItem implements ActionListener {
		public MyMenuItem(String text) {
			super(text);
			addActionListener(this);
		}

		public void actionPerformed(ActionEvent e) {
			if(e.getSource() == exit)
			{
				//close socket
				if(gui.getConnection().isConnected())
					gui.getConnection().close();
				if(gui.getLocalConnection().isConnected())
					gui.getLocalConnection().close();
				System.exit(0);
			}
		}

	}

	public void copyFile(String sourseDir, String destDir) throws IOException{

		InputStream in = new FileInputStream(new File (sourseDir));
		OutputStream out = new FileOutputStream(new File (destDir));

		// Copy the bits from instream to outstream
		byte[] buf = new byte[1024];
		int len;
		while ((len = in.read(buf)) > 0) {
			out.write(buf, 0, len);
		}
		in.close();
		out.close();
	}

	//subServer = true -> sub server, subServer = false -> loc server,
	public ArrayList<String> readAlgos(boolean subServer) throws IOException {
		ArrayList<String> algos = new ArrayList<String>();
		String path;
		if(subServer)
			path = "SubAlgos.config";
		else
			path = "locAlgos.config";

		BufferedReader br = new BufferedReader(new FileReader(path));
		try {
			StringBuilder sb = new StringBuilder();
			String line = br.readLine();
			while (line != null) {
				algos.add(line);
				line = br.readLine();
			}
		} 
		finally {
			br.close();
		}
		return algos;
	}


	private int createAddDialog(){
		filterW = new JTextField(15);
		configW = new JTextField(15);
		JPanel dialogPanel = new JPanel();

		BoxLayout layout = new BoxLayout(dialogPanel, BoxLayout.Y_AXIS);
		dialogPanel.setLayout(layout);

		JPanel row1 = new JPanel();
		FlowLayout fl = new FlowLayout();
		fl.setAlignment(FlowLayout.LEFT);
		row1.setLayout(fl);
		row1.add(new JLabel("Filter file path:                "));
		row1.add(filterW);

		row1.add(openFilterButton);
		dialogPanel.add(row1);

		row1 = new JPanel();
		row1.setLayout(fl);
		row1.add(new JLabel("Configuration file path:"));
		row1.add(configW);
		row1.add(openConfigButton);  
		dialogPanel.add(row1);

		int result = JOptionPane.showConfirmDialog(null, dialogPanel, 
				"Filter Configurations",  JOptionPane.OK_CANCEL_OPTION,JOptionPane.PLAIN_MESSAGE);
		return result;
	}




	//	private JButton openFilterButton, openConfigButton;



	public void disableLocalServerButtons(){
		runCVideoImage.setEnabled(false);
		runUCVideoImage.setEnabled(false);
		saveFVideoOrImage.setEnabled(false);
		stopSaveFVideoOrImage.setEnabled(false);
		openImage.setEnabled(false);
		openVideo.setEnabled(false);
		statisticLocalServer.setEnabled(false);
		addLoc.setEnabled(false); 
		createLoc.setEnabled(false); 
		chooseFiltersLoc.setEnabled(false);
		chooseAlgosLoc.setEnabled(false);
		removeFiltersLoc.setEnabled(false);
		removeAlgLoc.setEnabled(false);
		closeVideo.setEnabled(false);
	}

	public void disableSubServerButtons(){
		removeAlgSub.setEnabled(false);
		removeFiltersSub.setEnabled(false);
		chooseAlgosSub.setEnabled(false);
		chooseFiltersSub.setEnabled(false);
		createSub.setEnabled(false);
		addSub.setEnabled(false);
		exit.setEnabled(false);  
		runCFrontCamera.setEnabled(false);
		runCBottomCamera.setEnabled(false);
		runUCFrontCamera.setEnabled(false);
		runUCBottomCamera.setEnabled(false);
		saveUFCamera.setEnabled(false);
		saveUBCamera.setEnabled(false);  
		stopSaveUFCamera.setEnabled(false);	
		stopSaveUBCamera.setEnabled(false);
		saveFFCamera.setEnabled(false);
		saveFBCamera.setEnabled(false);
		stopSaveFFCamera.setEnabled(false);
		stopSaveFBCamera.setEnabled(false);
		statisticSubServer.setEnabled(false);
		camStartStream.setEnabled(false); 
		camStopStream.setEnabled(false);
	}

	public void enableLocalServerButtons(){
		runCVideoImage.setEnabled(true);
		runUCVideoImage.setEnabled(true);
		saveFVideoOrImage.setEnabled(true);
		stopSaveFVideoOrImage.setEnabled(true);
		openImage.setEnabled(true);
		openVideo.setEnabled(true);
		statisticLocalServer.setEnabled(true);
		addLoc.setEnabled(true); 
		createLoc.setEnabled(true); 
		chooseFiltersLoc.setEnabled(true);
		chooseAlgosLoc.setEnabled(true);
		removeFiltersLoc.setEnabled(true);
		removeAlgLoc.setEnabled(true);
		closeVideo.setEnabled(true);
		exit.setEnabled(true);
	}

	public void enableSubServerButtons(){
		removeAlgSub.setEnabled(true);
		removeFiltersSub.setEnabled(true);
		chooseAlgosSub.setEnabled(true);
		chooseFiltersSub.setEnabled(true);
		createSub.setEnabled(true);
		addSub.setEnabled(true);
		exit.setEnabled(true);  
		runCFrontCamera.setEnabled(true);
		runCBottomCamera.setEnabled(true);
		runUCFrontCamera.setEnabled(true);
		runUCBottomCamera.setEnabled(true);
		saveUFCamera.setEnabled(true);
		saveUBCamera.setEnabled(true);  
		stopSaveUFCamera.setEnabled(true);	
		stopSaveUBCamera.setEnabled(true);
		saveFFCamera.setEnabled(true);
		saveFBCamera.setEnabled(true);
		stopSaveFFCamera.setEnabled(true);
		stopSaveFBCamera.setEnabled(true);
		statisticSubServer.setEnabled(true);
		camStartStream.setEnabled(true); 
		camStopStream.setEnabled(true);
	}

	public void cleanUBC(){
		gui.unfilteredStreamCam2.removeAll();
		gui.unfilteredStreamCam2.revalidate();
		gui.unfilteredStreamCam2.repaint();	
	}
	public void cleanUFC(){
		gui.unfilteredStreamCam1.removeAll();
		gui.unfilteredStreamCam1.revalidate();
		gui.unfilteredStreamCam1.repaint();		
	}

	public void cleanFBC(){
		gui.filteredStreamCam2.removeAll();
		gui.filteredStreamCam2.revalidate();
		gui.filteredStreamCam2.repaint();		
	}
	public void cleanFFC(){
		gui.filteredStreamCam1.removeAll();
		gui.filteredStreamCam1.revalidate();
		gui.filteredStreamCam1.repaint();
	}
	public void cleanFImageVideo(){
		gui.filteredVideoStream.removeAll();
		gui.filteredVideoStream.revalidate();
		gui.filteredVideoStream.repaint();		
	}
	public void cleanUImageVideo(){
		gui.unfilteredVideoStream.removeAll();
		gui.unfilteredVideoStream.revalidate();
		gui.unfilteredVideoStream.repaint();		
	}




}