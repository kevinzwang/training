In this section, you will learn how to write Autonomous code.  As mentioned before, the Autonomous code is for the Autonomous period during the game.  While our bots are in the Autonomous period, It can score, leave the safezone, or even ruin other autos depending on how ambitous your autonomous will be.  It'll be fun.

Quick note, While you are writing your Autonomous Code and plotting your paths, consider your constraints and strategies.  Make sure you are utilizing the robot's potential to the fullest, making as many autonomous paths to give our alliance partners more strategies to work with.  Talk with your drive team to see what paths they would like to use during competition.  In general, make sure to have a strategic mindset whenever creating autons.

Now let's write some autonomous code!

## PathPlanner Tool
Our robots use the [`PathPlanner`](https://pathplanner.dev/home.html) tool for our autonomous as it is an easy and efficient way to set up pathways

So how exactly do we set this up and get the paths running?

## Setup in RobotContainer
Firstoff, we want to settup and import PathPlanner into RobotContainer. We will generally use this template to help us configure the PathPlanner tool.  Always reference to the [`PathPlanner Docs`](https://pathplanner.dev/home.html) whenever you are configuring the autonomous. 

Here when setting up the auto, we want to get constants such as robot's current positions and plug it into the methods, that way PathPlanner knows where our robot is
``` Java
public class DriveSubsystem extends SubsystemBase {
  public DriveSubsystem() {
    // All other subsystem initialization
    // ...

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }
}
```

## Creating Named Commands
Now that we have configured our autonomous, we need to create named commands to store the methods we will use in the PathPlanner tool(Example: Shooter Commands)
    
We always want to name our commands, that way we know which one to use when creating our autonomous
    
Then we want to run one actual command we created such as shooter so we would put the command name in there, and list the subsystem name as one of the parameters that way it knows what subsytem to use

``` Java
public class RobotContainer() {
    public RobotContainer() {
        // Subsystem initialization
        swerve = new Swerve();
        exampleSubsystem = new ExampleSubsystem();

        // Register Named Commands
        NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
        NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
        NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());
        //Example of a command we used on our bot
        NamedCommands.registerCommand("PassToOuttake", new PassToOuttake(intakeShooter));

        // Do all other initialization
        configureButtonBindings();

        // ...
    }
}
```
## Registering Autos On SmartDashboard
During competitions, strategies will change throughout matches, so you are going to want to change your autos depending on that strategy drive team comes up with

The question is how do we select auto right before a match?

It's pretty simple, all you have to do is register your autos onto SmartDashboard:

``` Java
   private List<Command> autoCommands = new ArrayList<Command>();
    private SendableChooser<Integer> autoSelector = new SendableChooser<Integer>();

    private boolean hasSetupAutos = false;
    private final String[] autoNames = new String[] {
            /* These are assumed to be equal to the AUTO ames in pathplanner */

            "1 Piece Shoot Auto", "2 Piece Shoot Auto", "3 Piece Shoot Auto",

            "Raise Arm", "Swing Arm", "Still Arm",

            "Auto Ruiner", "Safehouse Move", "No Moving",
    };
    DigitalInput[] autoSelectors = new DigitalInput[Math.min(autoNames.length, 10)];

    public RobotContainer() {
            registerAutoCommands();
            SmartDashboard.putData(autoSelector);
            SmartDashboard.setPersistent("SendableChooser[0]");
    }

```
What is happening here is we are putting our auto names in this array and creating a SmartDashboard selector to select our autos during a competition

## Running Autos In Code
Now that we have all of our Autos set up, we need to tell the robot to run and build the autonomous command

``` Java
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

  public Command getAutonomousCommand() {
    return autoSelectors.getSelected();
  }
}
```
What this code does is that it gets the autonomous we chose from SmartDashboard and plugs it into the getAutonomousCommand()

## creating autos!
Now that we have settup the neccesary code for auto, you can start creating autos and setting up paths.  One of the best parts of the autonomous creation process.

Open up your PathPlanner app, and open your robot project.  Once that is finished, your going to create a new path.  

That's about it, have fun creating your autonomous code!

## PathWeaver Section(Old Stuff)
One advantage of a characterized drivetrain is that it more effectively drives the robot to a precise location. This makes following autonomous paths a more viable option. All we need to do is tell the robot a specific path to follow and the robot will drive the path pretty accurately.

But how do we specify these paths? We use the `Trajectory` class along with WPILib's Pathweaver tool. For a more detailed description of how to use this class, check out WPIlib's [documentation](https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/trajectory-tutorial/index.html).

Let's write some autonomous code!

## Trajectories
The [`Trajectory`](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/trajectory/Trajectory.html) class creates a smooth path through a list of states. Each state contains information such as the position on the field, time elapsed, velocity, acceleration, pose, and the curvature of the path. Once you have built your paths using Pathweaver, you can use the [`fromPathweaverJson()`](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/trajectory/TrajectoryUtil.html#fromPathweaverJson(java.nio.file.Path)) method from [`TrajectoryUtil`](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/trajectory/TrajectoryUtil.html) to create a Trajectory from the built path. This will be the primary way in which we create our `Trajectory` objects.

If you want to create a trajectory given a list of points, you can use the [`TrajectoryGenerator`](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/trajectory/TrajectoryGenerator.html) class. There are many ways to generate a trajectory using this option, but I will only highlight the [`generateTrajectory()`](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/trajectory/TrajectoryGenerator.html#generateTrajectory(java.util.List,edu.wpi.first.wpilibj.trajectory.TrajectoryConfig)) method which uses a list of waypoints and a [`TrajectoryConfig`](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/trajectory/TrajectoryConfig.html) object. For the `TrajectoryConfig`, there are several constraints we can add. As an example, let's set the maximum speed, maximum acceleration, kinematics, voltage constraint, and whether or not the paths are inverted.

``` Java
TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAcceleration);
config.setKinematics(/* Your kinematics object */);
DifferentialDriveVoltageConstraint voltConstraint = new DifferentialDriveVoltageConstraint(/* Your SimpleMotorFeedForward Object */, /* Your kinematics object */, maximumVoltage);
// For the SimpleMotorFeedForward Object, construct a new object using the average of the kVolts, the average of the kV, and the average of the kA values.
config.addConstraint(voltConstraint);
config.setInverted(inverted);

List<Pose2d> waypoints = new List<Pose2d>();
/* Add your waypoints here, starting with the beginning of the path */
Trajectory trajectory = TrajectoryGenerator.generateTrajectory​(waypoints, config);
```

## Pathweaver Tool
We can use the Pathweaver tool to create waypoints and curves. You can find instructions for using the tool [here](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/pathweaver/creating-pathweaver-project.html).

## Following Pathweaver Paths
There are three steps to an autonomous path command:

1. Load the odometry so that the robot's position is at the start of the path.

2. Create a [`RamseteCommand`](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj2/command/RamseteCommand.html) to follow the trajectory. The `RamseteCommand` takes the following arguments:

    -  The trajectory to follow.

    - A `Supplier<Pose2d>` that gives the current pose of the robot. See [here](https://www.geeksforgeeks.org/supplier-interface-in-java-with-examples/) if you are unfamiliar with Suppliers.

    - A [`RamseteController`](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/controller/RamseteController.html) object.

    - Your kinematics object.

    - A `Biconsumer<Double, Double>` representing your command that drives the robot given left and right speeds in m/s. This will be your characterized drive method. See [here](https://www.geeksforgeeks.org/java-8-biconsumer-interface-in-java-with-examples/) if you are unfamiliar with Biconsumers.

    - The subsystems that will be required. This will probably just be the drivetrain.

3. Stop the robot.

It is highly recommended that you use the [`andThen()`](https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj2/command/Command.html#andThen(edu.wpi.first.wpilibj2.command.Command...)) method from the Command class to schedule these processes.

That's it! Pathweaver is a powerful and simple tool that lets you do incredible things. You could create a command group to turn on a feeder mechanism, drive a trajectory, align to a target, run a shooter mechanism, and drive to pick up more game elements.