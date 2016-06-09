package som.robot.starcraft.example.behavior;

import som.config.ConfigFile;
import som.math.Vector2D;
import som.robot.architecture.component.readonly.GlobalPose;
import som.robot.starcraft.component.mutable.StarcraftCommand;
import som.robot.starcraft.component.mutable.StarcraftMessagesMutable;
import som.robot.starcraft.component.readonly.StarcraftRobotState;
import som.robot.starcraft.component.readonly.StarcraftVisionFeatures;
import som.robot.starcraft.component.readonly.StarcraftWorldFeatures;
import som.robot.starcraft.module.StarcraftBehaviorBase;

public class MoveBehavior extends StarcraftBehaviorBase {

  public MoveBehavior(ConfigFile configFile) {
    super(configFile);
  }

  @Override
  protected void init() {
  }

  @Override
  public boolean run(StarcraftRobotState robotState, StarcraftVisionFeatures visionFeatures,
      StarcraftWorldFeatures worldFeatures, GlobalPose globalPose, StarcraftMessagesMutable messages,
      StarcraftCommand command) {
    System.out.println("Setting command!");
    command.goToGlobalPosition(new Vector2D(350, 200));
    return false;
  }

}
