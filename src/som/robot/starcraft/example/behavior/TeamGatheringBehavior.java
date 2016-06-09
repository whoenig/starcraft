package som.robot.starcraft.example.behavior;

import java.util.List;
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
import som.robot.starcraft.component.readonly.StarcraftTeammateWorldObject;
import som.robot.starcraft.component.readonly.StarcraftVisionFeatures;
import som.robot.starcraft.component.readonly.StarcraftWorldFeatures;
import som.robot.starcraft.component.readonly.StarcraftWorldObject;
import som.robot.starcraft.module.StarcraftBehaviorBase;

public class TeamGatheringBehavior extends StarcraftBehaviorBase {
  public Integer robID = 0;
  public Integer target = 0;
	
  public TeamGatheringBehavior(ConfigFile configFile) {
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
      fsm.addState(State.LookingForEnemy);
      fsm.addState(State.AttackingEnemy);
      fsm.addState(State.Gather);
      fsm.addState(State.WaitForTeam);
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
    	LOGGER.info("respawn with ID " + Integer.toString(robID) + " " + Double.toString(globalPose.getPosition().getX()) +  " " + Double.toString(globalPose.getPosition().getY()));
    	
    }

    // Pick the nearest object in the world model
    StarcraftWorldObject resource = null;
    StarcraftWorldObject enemy = null;

    double bestDistance = Double.MAX_VALUE;
    for (StarcraftWorldObject object : worldFeatures.getStarcraftWorldObjects(StarcraftObjectType.RESOURCE)) {
      double distance = globalPose.getPosition().distanceTo(object.getGlobalPose().getPosition());
      if (distance < bestDistance) {
        bestDistance = distance;
        resource = object;
      }
    }
    List<StarcraftWorldObject> resources = worldFeatures.getStarcraftWorldObjects(StarcraftObjectType.RESOURCE);

    List<StarcraftTeammateWorldObject> teammates = worldFeatures.getStarcraftTeammateWorldObjects();

    
    
    for (StarcraftWorldObject object : worldFeatures.getStarcraftWorldObjects(StarcraftObjectType.OPPONENT_ROBOT)) {
        double distance = globalPose.getPosition().distanceTo(object.getGlobalPose().getPosition());
        if (distance < bestDistance) {
          bestDistance = distance;
          enemy = object;
        }
      }
    
    try {
      fsm.startLoop(timestamp);
      while (fsm.isRunning()) {

        if (fsm.inState(State.Idle)) {
          // If the robot is idle, head to a random location
          fsm.transition(State.Gather, "Head to gathering location");
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
        else if (fsm.inState(State.Gather)) {
        	Vector2D meetingpoint = new Vector2D(800, 100);
        	Integer teamsize = 3, distlimit = 50;
        	
            double mydist = globalPose.getPosition().distanceTo(meetingpoint);
        	
        	// count number of teammates
        	int count = 0;
            for (StarcraftWorldObject object : worldFeatures.getStarcraftTeammateWorldObjects()) {
                double distance = object.getGlobalPose().getPosition().distanceTo(meetingpoint);
                if (distance < distlimit) {
                  count++;
                }
              }
            
             if (count >= teamsize && mydist < distlimit) {
                // Tell our teammates which resource we're heading to
                messages.getMessageToBeSent().setMessage(Integer.toString(resource.getID()));             
            }
            
            if (mydist < distlimit) {
                fsm.transition(State.WaitForTeam, "Waiting for team");
                continue;                            	
            }
            LOGGER.info(Double.toString(mydist));
            
            command.goToGlobalPosition(meetingpoint);
           
            // Otherwise, keep heading to the location
            fsm.endLoop();
          }
        else if (fsm.inState(State.WaitForTeam)) {
            for (StarcraftTeammateWorldObject teammate : teammates) {
              // Ignore teammate with high IDs
              if (teammate.getID() >= robotState.getID()) {
                continue;
              }
              // check for leaders message
              if (teammate.getLastMessage().length() > 0) {
                Integer resourceID = Integer.parseInt(teammate.getLastMessage());
                if (teammate.getLastMessageTimestamp() > robotState.getTimestamp() - 1000 ){
                	
                	for(StarcraftWorldObject res : resources){
                		if (res.getID() == resourceID){
                			targetLocation = res.getGlobalPose().getPosition();
                		}
                			
                	}

                    fsm.transition(State.HeadingToResource, "Heading to a resource");
                    continue;                	
                }
              }
            }        	
            

            fsm.endLoop();
          }
        else if (fsm.inState(State.LookingForEnemy)) {
            // If the robot has chosen a resource, head towards it
            if (enemy != null) {
              fsm.transition(State.AttackingEnemy, "Found enemy");
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
        
        
        else if (fsm.inState(State.AttackingEnemy)) {

            // If the robot loses sight of any resources, enter the idle state
            if (enemy == null) {
              fsm.transition(State.Idle, "Lost sight of resource");
              continue;
            }

            //if (!targetLocation.equals(enemy.getGlobalPose().getPosition())) {
              // Head towards the resource
              LOGGER.fine("I am at: " + globalPose.getPosition());
              LOGGER.fine("Heading to resource " + resource.getID() + " at " + resource.getGlobalPose().getPosition());
              LOGGER.fine("State of enemy, visible: " + resource.isVisible() + ", suspicious: "
                  + resource.isSuspicious() + ", valid: " + resource.isValid());
              //command.goToGlobalPosition(enemy.getGlobalPose().getPosition());
              command.attack(enemy);
            //}
            // If the robot arrived but doesn't have a resource, enter the idle state
            //else if (robotState.isIdle()) {
            //  fsm.transition(State.Idle, "Arrived at location but did not pick up resource");
            //  continue;
            //}

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
    Idle, HeadingToRandomLocation, HeadingToResource, HeadingHome, LookingForEnemy, WaitForTeam, AttackingEnemy, Gather
  }

  final static Logger LOGGER = Logger.getLogger(GatheringBehavior.class.getName());

  Random random;
  Vector2D targetLocation;
}
