package frigolf;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;

/**
 * Moves the LBR in a start position, creates an FRI-Session and executes a
 * PositionHold motion with FRI overlay. During this motion joint angles and
 * joint torques can be additionally commanded via FRI.
 */
public class FRIGolf extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;

    @Inject
    @Named("GolfPutter")
    private Tool gripper;
    
    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "192.170.10.10";
        gripper.attachTo(_lbr.getFlange());       
    }

    @Override
    public void run()
    {
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        // for torque mode, there has to be a command value at least all 5ms
        friConfiguration.setSendPeriodMilliSec(5);
        friConfiguration.setReceiveMultiplier(1);

        getLogger().info("Tentative de connexion FRI à l'ordinateur...");
        
        FRISession friSession = new FRISession(friConfiguration);

        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(10, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error("Le robot ne peut pas se connecter à l'ordinateur après 10 secondes. Branchements OK ?");
            friSession.close();
            return;
        }

        getLogger().info("Connexion FRI avec l'ordinateur établie");
        
        int powerChoice = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Vérifiez qu'aucune personne ou obstacle ne se trouve à moins de 2m du robot puis confirmer la mise sous tension du robot", "Mettre sous tension", "Annuler");
        ClientCommandMode mode = ClientCommandMode.POSITION;

        if (powerChoice == 0) {
        	getLogger().info("Déverouillage des freins et mise sous tension en cours");
        	
            // start PositionHold with overlay
        	double stiffness = 300.;
        	JointImpedanceControlMode ctrMode = new JointImpedanceControlMode(stiffness, stiffness, stiffness, stiffness, stiffness, stiffness, stiffness);

        	try {
	            PositionHold posHold = new PositionHold(ctrMode, -1, TimeUnit.SECONDS);
	            FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession, mode);
	        	getLogger().info("Le robot est prêt !");
	            _lbr.move(posHold.addMotionOverlay(jointOverlay));
        	}
        	catch(final CommandInvalidException e) {
            	getLogger().error("Le robot s'est arrêté automatiquement. Veuillez DECOCHER puis RECOCHER FriGolf dans le menu Application avant de relancer avec la flèche verte >");
        	}
        }
        else {
        	getLogger().error("Mise sous tension avortée. Veuillez décocher et recocher FriGolf dans le menu Application avant de la relancer avec la flèche verte >");
        }

        // done
        friSession.close();
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final FRIGolf app = new FRIGolf();
        app.runApplication();
    }

}
