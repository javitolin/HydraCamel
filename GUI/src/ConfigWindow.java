


import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.Scanner;
import java.util.Vector;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.SwingConstants;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

public class ConfigWindow extends JFrame {

	JPanel labelsPanel;
	JPanel slidersPanel;
	JPanel txtFieldsPanel;
	JLabel[] labels;
	JSlider[] sliders;
	JTextField[] txtFields;

	JButton saveButton;
	JButton resetButton;

	String[] defaultProperties;
	int[] defaultValues;
	int numOfLines = -1;
	String filterPath;
	private ClientGUI gui;
	JPanel row1;
	JPanel row2 = new JPanel();
	JPanel dialogPanel; 

	public ConfigWindow(String filterPath, ClientGUI gui) throws IOException {
		this.filterPath = filterPath;
		this.gui = gui;
		
		JPanel dialogPanel = new JPanel();
		row1 = new JPanel();
		row2 = new JPanel();
		
		BoxLayout layout = new BoxLayout(dialogPanel, BoxLayout.Y_AXIS);
 	    dialogPanel.setLayout(layout);
 	      
 	 
 	    FlowLayout fl = new FlowLayout();
 	    fl.setAlignment(FlowLayout.LEFT);
 	    row1.setLayout(fl);
		
		setMisc();
		setPanels();
		
	    dialogPanel.add(row1);
	    
	    setButtons();     
		    
		dialogPanel.add(row2);
	    this.add(dialogPanel);

	}

	private void setMisc() throws IOException {
		numOfLines = readConfigFile(System.getProperty("user.dir")+"/" +  filterPath + ".config");
		System.out.println("ASdsadsadsadsad\t "+System.getProperty("user.dir")+"/" +  filterPath + ".config");
		labels = new JLabel[numOfLines];
		sliders = new JSlider[numOfLines];
		txtFields = new JTextField[numOfLines];
	}

	private void setPanels() {
		labelsPanel = new JPanel(new GridLayout(numOfLines, 1, 0, 9));
		slidersPanel = new JPanel(new GridLayout(numOfLines, 1, 0, 10));
		txtFieldsPanel = new JPanel(new GridLayout(numOfLines, 1, 0, 5));

		for(int i=0; i<numOfLines; i++)
		{
			Event e = new Event();

			labels[i] = new JLabel(defaultProperties[i]);
			labelsPanel.add(labels[i]);

			sliders[i] = new JSlider(SwingConstants.HORIZONTAL, 0, defaultValues[i]*3, defaultValues[i]);
			sliders[i].addChangeListener(e);
			slidersPanel.add(sliders[i]);

			txtFields[i] = new JTextField(String.valueOf(defaultValues[i]));
			txtFields[i].addActionListener(e);
			txtFieldsPanel.add(txtFields[i]);
		}

		row1.add(labelsPanel);
		row1.add(slidersPanel);
		row1.add(txtFieldsPanel);
	}

	private void setButtons() {
		Event e = new Event();

		saveButton = new JButton("Apply");
		saveButton.addActionListener(e);
		saveButton.setFont(new Font("Dialog",Font.BOLD,10));
		row2.add(saveButton);

		resetButton = new JButton("Reset");
		resetButton.addActionListener(e);
		resetButton.setFont(new Font("Dialog",Font.BOLD,10));
		row2.add(resetButton);
	}

	private int readConfigFile(String filename) throws IOException {
		int count = count(filename);
		Scanner sc = new Scanner(new File(filename));
		Vector<String> dpVector = new Vector<String>();
		Vector<Integer> dvVector = new Vector<Integer>();
		int k = 0;
		while(sc.hasNext())
		{
			String[] strings = sc.nextLine().split(" ");
			dpVector.add(strings[0]);
			dvVector.add(new Integer(strings[1]));
			/*defaultProperties[k] = strings[0];
			defaultValues[k] = new Integer (strings[1]);*/
			k++;
		}
		defaultProperties = new String[dpVector.size()];
		defaultValues = new int[dpVector.size()];
		for(int i = 0; i<defaultValues.length; i++){
			defaultValues[i] = dvVector.get(i).intValue();
			defaultProperties[i] = dpVector.get(i);
		}
				

		sc.close();
		return count;
	}

	public int count(String filename) throws IOException {
		InputStream is = new BufferedInputStream(new FileInputStream(filename));
		try {
			byte[] c = new byte[1024];
			int count = 0;
			int readChars = 0;
			boolean empty = true;
			while ((readChars = is.read(c)) != -1) {
				empty = false;
				for (int i = 0; i < readChars; ++i)
					if (c[i] == '\n')
						++count;
			}
			return (count == 0 && !empty) ? 1 : count;
		} finally {
			is.close();
		}
	}

	public class Event implements ActionListener,ChangeListener {
		public void actionPerformed(ActionEvent e) {
			for(int i=0; i<numOfLines; i++)
				if(e.getSource() == txtFields[i])
					sliders[i].setValue(Integer.parseInt(txtFields[i].getText()));

			if(e.getSource() == saveButton)
			{
				PrintWriter writer = null;
				try {
					writer = new PrintWriter(filterPath + ".config", "UTF-8");
				} catch (FileNotFoundException | UnsupportedEncodingException ex) {
					ex.printStackTrace();
				}

				if(writer != null)
				{
					for(int i=0; i<numOfLines; i++)
						writer.println(labels[i].getText()+" "+sliders[i].getValue());
					writer.close();
				}
				
				if(gui.conConfigOpen == 0)
					gui.getConnection().changeConfig(System.getProperty("user.dir")+"/" +  filterPath + ".config");
				else 
					if (gui.conConfigOpen == 1)
					gui.getLocalConnection().changeConfig(System.getProperty("user.dir")+"/" +  filterPath + ".config");				
				gui.closeConfigWindow();
			
			}

			else if(e.getSource() == resetButton)
			{
				for(int i=0; i<numOfLines; i++)
				{
					sliders[i].setValue(defaultValues[i]);
					txtFields[i].setText(String.valueOf(defaultValues[i]));
				}

				PrintWriter writer = null;
				try {
					writer = new PrintWriter(filterPath + ".config", "UTF-8");
				} catch (FileNotFoundException | UnsupportedEncodingException ex) {
					ex.printStackTrace();
				}

				if(writer != null)
				{
					for(int i=0; i<numOfLines; i++)
						writer.println(defaultProperties[i] + " " + defaultValues[i]);
					writer.close();
				}
			
				//TODO:: do i realy need this???     conc.changeConfig();
			}
		}

		public void stateChanged(ChangeEvent e) {
			for(int i=0; i<numOfLines; i++)
				if(e.getSource() == sliders[i])
					txtFields[i].setText(""+sliders[i].getValue());	
		}
	}
}