package som.robot.starcraft.runner;

import java.util.Scanner;

import som.config.ConfigFile;
import som.robot.starcraft.example.behavior.MoveBehavior;
import som.robot.starcraft.example.behavior.GatheringBehavior;

import som.robot.starcraft.module.StarcraftBehaviorBase;
import som.robot.starcraft.module.StarcraftFactory;

public class RunStarcraftJavaRobot {

  public static void main(String[] args) throws Exception {
    ConfigFile configFile = new ConfigFile("config", "config.txt");
    // StarcraftBehaviorBase behavior = new MoveBehavior(configFile);
    StarcraftBehaviorBase behavior = new GatheringBehavior(configFile);
    // StarcraftBehaviorBase behavior = new CoordinatedGatheringBehavior(configFile);

    // Create the factor and connect to the server
    StarcraftFactory factory = new StarcraftFactory(configFile, behavior);

    // Keep running until <enter>
    Scanner scanner = new Scanner(System.in);
    System.out.println("Press <enter> to end");
    scanner.nextLine();
    scanner.close();

    // End the agent
    System.out.println("Ending.");
    factory.getStarcraftAgent().stopRunning();
    System.out.println("Ended.");
  }

}
