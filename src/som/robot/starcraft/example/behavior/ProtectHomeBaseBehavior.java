package som.robot.starcraft.example.behavior;

import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import som.config.ConfigFile;
import som.math.Vector2D;
import som.robot.architecture.component.readonly.GlobalPose;
import som.robot.architecture.module.FSM;
import som.robot.architecture.module.FSM.FSMException;
import som.robot.starcraft.component.mutable.StarcraftCommand;
import som.robot.starcraft.component.mutable.StarcraftMessagesMutable;
import som.robot.starcraft.component.readonly.StarcraftObjectType;
import som.robot.starcraft.component.readonly.StarcraftRobotState;
import som.robot.starcraft.component.readonly.StarcraftVisionFeatures;
import som.robot.starcraft.component.readonly.StarcraftWorldFeatures;
import som.robot.starcraft.component.readonly.StarcraftWorldObject;
import som.robot.starcraft.module.StarcraftBehaviorBase;

public class ProtectHomeBaseBehavior extends StarcraftBehaviorBase {
  public Integer robID = 0;
	
  public ProtectHomeBaseBehavior(ConfigFile configFile) {
    super(configFile);

    FSM.LOGGER.setLevel(Level.SEVERE);

    random = new Random();
    targetLocation = getHomeLocation();
  }

  @Override
  protected void init() {
    // Initialize the FSM
    fsm.reset();
    try {
      fsm.addState(State.Idle);
      fsm.addState(State.HeadingHome);
      fsm.addState(State.LookingForEnemy);
      fsm.finishInit(State.Idle, System.currentTimeMillis());
    }
    catch (FSMException e) {
      LOGGER.warning("Error initializing FSM: " + e.getMessage());
      fsm.reset();
    }
  }

  @Override
  public boolean run(StarcraftRobotState robotState, StarcraftVisionFeatures visionFeatures,
      StarcraftWorldFeatures worldFeatures, GlobalPose globalPose, StarcraftMessagesMutable messages,
      StarcraftCommand command) {
    long timestamp = robotState.getTimestamp();
    
    if(robotState.getID() != robID) {
    	robID = robotState.getID();
    	LOGGER.info("respawn with ID " + Integer.toString(robID));
    }

    // Pick the nearest object in the world model
    StarcraftWorldObject enemy = null;
    
    try {
      fsm.startLoop(timestamp);
      while (fsm.isRunning()) {

        if (fsm.inState(State.Idle)) {
          // If the robot is idle, head to a home location
          fsm.transition(State.HeadingHome, "Head to home location");
          continue;
        }
        else if (fsm.inState(State.HeadingHome)) {
            // Head home
            if (fsm.isNewState()) {
              command.goToGlobalPosition(getHomeLocation());
            }
            // If the robot arrived, check if enemy nearby
            else if (robotState.isIdle()) { 
              Vector2D rpose = globalPose.getPosition();
              Vector2D hbase = getHomeLocation();
              
              if ( rpose.distanceTo(hbase) < 200 )
              {
            	fsm.transition(State.LookingForEnemy, "Looking for enemy");
              	continue;
              }
            }

            // Otherwise, keep heading home
            fsm.endLoop();
          }
        else if (fsm.inState(State.LookingForEnemy)) {
        	// find enemy
            double bestDistance = 300;
            for (StarcraftWorldObject object : worldFeatures.getStarcraftWorldObjects(StarcraftObjectType.OPPONENT_ROBOT))
            {
                double distance = globalPose.getPosition().distanceTo(object.getGlobalPose().getPosition());
                if (distance < bestDistance)
                {
                  bestDistance = distance;
                  enemy = object;
                }
            }
            
            // if found enemy nearby, kill it
            if (enemy != null)
            {
                System.out.println("found enemy, killing");
                command.attack(enemy);
            }
            else
            	fsm.transition(State.HeadingHome, "Go to home location");
            
            // Otherwise, keep heading to the random location
            fsm.endLoop();
          }

        else {
          LOGGER.warning("FSM entered an invalid state.");
          fsm.transition(State.Idle, "Invalid state");
        }
      }
    }
    catch (FSMException e) {
      if (fsm.isRunning()) {
        try {
          fsm.endLoop();
        }
        catch (FSMException e2) {
          LOGGER.severe("Error during fsm.endLoop within catch.");
        }
      }
    }
    fsm.printTransitions(true);

    return false;
  }

  public enum State implements FSM.State {
    Idle, HeadingHome, LookingForEnemy
  }

  final static Logger LOGGER = Logger.getLogger(GatheringBehavior.class.getName());

  Random random;
  Vector2D targetLocation;
}
