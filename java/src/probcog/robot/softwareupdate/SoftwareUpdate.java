package magic.softwareupdate;

import java.io.*;
import javax.swing.*;
import java.awt.*;
import java.util.*;

import lcm.lcm.*;

import april.util.*;
import april.config.*;

import magic.util.*;
import magic.lcmtypes.*;
/**
 * Class which handles software updates to robots. Includes a GUI.
 *
 *  TODO:
 *   - Add Central GUI panel which shows terminal output of update progress
 *   - Add an 'Update all outofdate' button
 *   - Add a confirmation dialog before initiating a restart?
 */
public class SoftwareUpdate implements ParameterListener, LCMSubscriber
{
    private static final boolean verbose = false;

    static LCM lcm = LCM.getSingleton();

    JFrame jf;
    ParameterGUI pg;

    // Tracks all the robots which have notified us they exist over wifi

    String updateScript, setupScript, restartScript, shutdownScript;
    String magicHome;


    JList jlist;
    RobotListModel model;

    ArrayList<RobotRecord> robots;
    HashMap<Integer,Integer> indexIDMap;

    // Expected locations of the magic build.xml files.
    // Trailing space due to bug in StringUtil
    private static final String[] BUILD_PATHS =  {"$MAGIC_HOME",
                                                  "$HOME/magic/java"};


    public SoftwareUpdate()
    {

        Config config = RobotUtil.getConfig();
        magicHome = getMagicHome();
        updateScript = magicHome + "/scripts/install.py";
        setupScript  = magicHome + "/scripts/setupRobot.py";
        restartScript  = magicHome + "/scripts/restart.py";
        shutdownScript  = magicHome + "/scripts/shutdown.py";

        robots = new ArrayList<RobotRecord>();
        indexIDMap = new LinkedHashMap<Integer,Integer>();

        setupPG();
        setupGUI();

        lcm.subscribe("ROBOT_STATUS_.*", this);
    }

    private String getMagicHome()
    {
        return StringUtil.replaceEnvironmentVariables("$HOME/magic");
    }

    private void setupPG()
    {
        pg = new ParameterGUI();

        pg.addString("curHash","Machine version","");
        pg.addString("prefix","IP Prefix","192.168.10");
        pg.addCheckBoxes("enableShudown", "Enable Shutdown", false);
        pg.addButtons("refresh", "Refresh", "updateSel","Update Selected","setupSel","Setup Selected",
                      "restartSel","Restart Selected", "shutdownSel", "Shutdown Selected");
        pg.addListener(this);
    }

    private void setupGUI()
    {
        model = new RobotListModel();
        jlist = new JList(model);
        jlist.setSelectionMode(ListSelectionModel.MULTIPLE_INTERVAL_SELECTION);

        JFrame jf = new JFrame("Robot Software Update GUI");
        jf.setLayout(new BorderLayout());
        jf.add(jlist, BorderLayout.NORTH);
        jf.add(pg, BorderLayout.SOUTH);

        jf.setSize(700,400);
        jf.setVisible(true);
    }

    public void parameterChanged(ParameterGUI pg, String name)
    {
        if (name.equals("refresh")) {
            refreshHashes();
        } else { // setup or update
            actionSelected(name);
        }

    }

    private void actionSelected(String action)
    {
        RobotRecord selected[] = getSelected();
        if (selected.length == 0) {
            System.out.println("NFO: No robot selected!");
            return;
        }

        String ips = "";
        for (int i = 0; i < selected.length; i++) {
            ips += " ";

            ips += pg.gs("prefix") + "."+selected[i].id;
        }
        if (action.equals("updateSel")) {
            callScript(updateScript +" "+ ips);
            refreshHashes();
        } else if (action.equals("setupSel")) {
            callScript(setupScript +" "+ ips);
        } else if (action.equals("restartSel"))
            callScript(restartScript +" "+ ips);
        else if (action.equals("shutdownSel") && pg.gb("enableShudown")){
            callScript(shutdownScript +" "+ ips);
            pg.sb("enableShudown", false);
        }
    }

    private void callScript(String cmd)
    {
        System.out.println("NFO: Attempting to run \n$> "+cmd);
        System.out.println("NFO: Please be patient. This may take a few moments");
        int exit;
        try{
            Process p = Runtime.getRuntime().exec(cmd);
            BufferedReader stdin = new BufferedReader(new InputStreamReader(p.getInputStream()));

            while (true) {
                String line = stdin.readLine();
                if (line == null) {
                    try{
                        exit = p.exitValue();
                        break;
                    }catch(IllegalThreadStateException e){
                        TimeUtil.sleep(5);
                        continue;
                    }
                }
                System.out.println(line);
            }
            p.destroy();

        } catch (IOException e) {
            System.out.println("WRN: Failed to read output from cmd "+cmd);
            e.printStackTrace();
        }
        System.out.println("NFO: success.");
    }

    // Poll each robot, and determine what the version is
    private void  refreshHashes()
    {
        System.out.println("DBG: Refreshing");
        RobotRecord selected[] = getSelected();
        if (selected.length == 0) {
            System.out.println("NFO: No robot selected!");
            return;
        }

        String mhome =  StringUtil.replaceEnvironmentVariables("/home/$USER/magic");

        for (RobotRecord r : selected) {
            callScript("scp -i "+mhome+"/config/magic_rsa april@"+
                       pg.gs("prefix")+"."+r.id+":/home/april/magic/git-hash /tmp/hash"+r.id);
            try {
                BufferedReader bin = new BufferedReader(new FileReader("/tmp/hash"+r.id));
                String version = bin.readLine();
                bin.close();
                if (version != null)
                    model.updateVersion(r.id, version);
            } catch (Exception e){
                System.out.println("Unable to load hash from file: /tmp/hash"+r.id);
            }
        }
    }

    public void run()
    {
        while(true){
            updateGitHash();
            TimeUtil.sleep(200);
            model.updateAll();
            TimeUtil.sleep(200);
            model.updateAll();
        }
    }

    /**
     * TODO:
     *  Should probably create a script which generates the current hash
     *  as a combination between the git commit sha1, and a hash of the git diff
     *  Then, we could call that script from both install.py, and here
     */
    private void updateGitHash()
    {
        String tempFile = "/tmp/git-hash";
        String cmd = magicHome+"/scripts/gen-git-hash "+tempFile;

        try{
            // Go to magicHome, and find the current git hash
            Process p = Runtime.getRuntime().exec(cmd);
            BufferedReader stdin = new BufferedReader(new InputStreamReader(p.getInputStream()));
            try{
                p.waitFor();
            }catch(InterruptedException e){}

            // Find the mac address which is stored in the file at 'path'
            BufferedReader bin = new BufferedReader(new FileReader(tempFile));
            String hash = bin.readLine();
            bin.close();

            pg.ss("curHash", hash);
            p.destroy();
        }catch(IOException e){
            System.out.println("WRN: Unable to find git hash in "+magicHome);
            e.printStackTrace();
        }
    }

    public RobotRecord[] getSelected()
    {
        int selections[]  = jlist.getSelectedIndices();

        RobotRecord records [] = new RobotRecord[selections.length];
        for (int i = 0; i < selections.length; i++) {
            records[i] = robots.get(selections[i]);
        }

        return records;
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            if (channel.startsWith("ROBOT_STATUS_")) {
                int robotid = CommandUtil.getSuffixID(channel);
                if (robotid == -1)
                    return;

                robot_status_t status = new robot_status_t(ins); // Not needed
                model.addRobot(robotid);
            }
        }catch(IOException e){}
    }
    public class RobotRecord
    {
        int id;
        String version = "<not set>";
        long lastHeard;

        public String toString()
        {
            return "RobotID = " + id +
                " time  = " +
                ((System.currentTimeMillis() -  lastHeard) / 1000.0) +
                " robot version = " + version;
        }
    }

    public class RobotListModel extends AbstractListModel
    {
        public Object getElementAt(int index)
        {
            return robots.get(index);
        }

        public int getSize()
        {
            return robots.size();
        }

        public void addRobot(int id)
        {
            Integer idx = indexIDMap.get(id);
            boolean create = false;

            RobotRecord rrecord = null;
            if (idx == null) { // Then we haven't seen this robot yet
                create = true;
                rrecord = new RobotRecord();
                rrecord.id = id;

                idx = robots.size();
                indexIDMap.put(id, idx);
                robots.add(rrecord);
                if (verbose)
                    System.out.println("Added new robot with idx="+idx+" rsize="
                                       + robots.size());
            } else {
                rrecord = robots.get(idx);
                if (verbose)
                    System.out.println("Found old robot with idx="+idx+" rsize="
                                       +robots.size());
            }

            // In either case, also need to update last timestamp
            rrecord.lastHeard = System.currentTimeMillis();

            // Fire an event changed so the display will change
            if (create)
                this.fireIntervalAdded(this, idx, idx);
            else
                this.fireContentsChanged(this, idx, idx);

        }

        public void updateVersion(int id, String version)
        {
            Integer idx = indexIDMap.get(id);
            if (idx == null)
                return;
            RobotRecord rrecord = robots.get(idx);
            rrecord.version = version;

            this.fireContentsChanged(this, idx, idx);
        }

        public void updateAll()
        {
            this.fireContentsChanged(this, 0, robots.size()-1);
        }

        public void clear()
        {
            int lastidx = robots.size()-1;
            robots.clear();
            if (lastidx > 0)
                this.fireIntervalRemoved(this, 0, lastidx);
        }
    }

    public static void main(String args[])
    {
        new SoftwareUpdate().run();
    }
}
