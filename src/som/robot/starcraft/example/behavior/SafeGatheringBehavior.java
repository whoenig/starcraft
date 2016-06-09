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

public class SafeGatheringBehavior extends StarcraftBehaviorBase {

  public SafeGatheringBehavior(ConfigFile configFile) {
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
      fsm.addState(State.HeadingToRandomLocation);
      fsm.addState(State.HeadingToResource);
      fsm.addState(State.HeadingHome);
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
    
    System.out.println("ID:" + robotState.getID());
    
    // First, we check if an opponent is close by, if so attack
    StarcraftWorldObject enemy = null;
    double bestDistance = Double.MAX_VALUE;
    
    for (StarcraftWorldObject object : worldFeatures.getStarcraftWorldObjects(StarcraftObjectType.OPPONENT_ROBOT)) {
        double distance = globalPose.getPosition().distanceTo(object.getGlobalPose().getPosition());
        if (distance < bestDistance) {
          bestDistance = distance;
          enemy = object;
        }
      }
    
    if (enemy != null && bestDistance < 100) {
    	command.attack(enemy);
    	System.out.println("Enemy nearby -> attack");
    	return false;
    }
    
    // Pick the nearest object in the world model
    StarcraftWorldObject resource = null;
    bestDistance = Double.MAX_VALUE;
    for (StarcraftWorldObject object : worldFeatures.getStarcraftWorldObjects(StarcraftObjectType.RESOURCE)) {
      double distance = globalPose.getPosition().distanceTo(object.getGlobalPose().getPosition());
      if (distance < bestDistance) {
    	  boolean guardiansAreClose = false;
    	  for (StarcraftWorldObject object2 : worldFeatures.getStarcraftWorldObjects(StarcraftObjectType.GUARDIAN)) {
    	      double distance2 = object2.getGlobalPose().getPosition().distanceTo(object.getGlobalPose().getPosition());
//    	      System.out.println("GuardDist: " + distance2);
    	      if (distance2 < 100) {
    	    	  guardiansAreClose = true;
    	      }
    	    }
    	  
    	if (!guardiansAreClose) {
    		bestDistance = distance;
    		resource = object;
    	}
      }
    }
    
    if (resource != null) {
    	System.out.println("Found resource with no guardians!");
    }
    

    try {
      fsm.startLoop(timestamp);
      while (fsm.isRunning()) {

        if (fsm.inState(State.Idle)) {
          // If the robot is idle, head to a random location
          fsm.transition(State.HeadingToRandomLocation, "Head to random location");
          continue;
        }

        else if (fsm.inState(State.HeadingToRandomLocation)) {
          // If the robot has chosen a resource, head towards it
          if (resource != null) {
            fsm.transition(State.HeadingToResource, "Found a resource");
            continue;
          }

          // Set a random target if we just entered the state
          if (fsm.isNewState()) {
            int randomX = random.nextInt(getWorldMaxX() - getWorldMinX() + 1) + getWorldMinX();
            int randomY = random.nextInt(getWorldMaxY() - getWorldMinY() + 1) + getWorldMinY();
            targetLocation = new Vector2D(randomX, randomY);
            command.goToGlobalPosition(targetLocation);
            LOGGER.info("Heading to random location: (" + randomX + ", " + randomY + ")");
          }
          // If the robot is idle, switch to the idle state
          else if (robotState.isIdle() && (fsm.getTimeInState() > 500)) {
            fsm.transition(State.Idle, "Arrived at location");
            continue;
          }

          // Otherwise, keep heading to the location
          fsm.endLoop();
        }

        else if (fsm.inState(State.HeadingToResource)) {
          // If the robot is carrying a resource, head home
          if (robotState.hasPickedUpResource()) {
            fsm.transition(State.HeadingHome, "Picked up resource");
            continue;
          }
          // If the robot loses sight of any resources, enter the idle state
          if (resource == null) {
            fsm.transition(State.Idle, "Lost sight of resource");
            continue;
          }

          if (!targetLocation.equals(resource.getGlobalPose().getPosition())) {
            // Head towards the resource
            LOGGER.fine("I am at: " + globalPose.getPosition());
            LOGGER.fine("Heading to resource " + resource.getID() + " at " + resource.getGlobalPose().getPosition());
            LOGGER.fine("State of resource, visible: " + resource.isVisible() + ", suspicious: "
                + resource.isSuspicious() + ", valid: " + resource.isValid());
            command.goToGlobalPosition(resource.getGlobalPose().getPosition());
          }
          // If the robot arrived but doesn't have a resource, enter the idle state
          else if (robotState.isIdle()) {
            fsm.transition(State.Idle, "Arrived at location but did not pick up resource");
            continue;
          }

          // Otherwise, keep heading to the random location
          fsm.endLoop();
        }
        else if (fsm.inState(State.HeadingHome)) {
          // If the robot is not carrying a resource, enter the idle state
          if (!robotState.hasPickedUpResource()) {
            fsm.transition(State.Idle, "No longer carrying resource");
            continue;
          }

          // Head home
          if (fsm.isNewState()) {
            command.goToGlobalPosition(getHomeLocation());
          }
          // If the robot arrived, enter the idle state
          else if (robotState.isIdle()) {
            fsm.transition(State.Idle, "Arrived at home location");
            continue;
          }

          // Otherwise, keep heading home
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
    Idle, HeadingToRandomLocation, HeadingToResource, HeadingHome
  }

  final static Logger LOGGER = Logger.getLogger(GatheringBehavior.class.getName());

  Random random;
  Vector2D targetLocation;
}
