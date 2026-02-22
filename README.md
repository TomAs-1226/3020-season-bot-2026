# Team 3020 Robot Code

Hello Team 3020 this is the scafolding code for the basic subsystems implemented with out the **SWERVE** subsystem for this robot 

### The Mechanisms

| Part | What It Does | Motors |
|------|--------------|--------|
| **Intake Arm** | Swings down to the floor | 1x Kraken X60 (CAN 8) |
| **Intake Rollers** | Grabs game pieces | 1x Kraken X44 (CAN 5) |
| **Feeder** | Pushes pieces to shooter | 2x Kraken X60 (CAN 1, 2) |
| **Shooter** | Launches pieces at target | 4x Kraken X60 (CAN 3, 4, 6, 7) |

## Controller Layout (PS5)

```
       [L1]                [R1]          L1 = Shooter forward
       [L2]                [R2]          R1 = Shooter reverse

  [D-PAD]     [OPTIONS]  [TRIANGLE]      Triangle = Intake rollers only
    UP                    [SQUARE]        Square = Auto intake (arm + rollers)
  L    R                   [CIRCLE]       Circle = Eject
   DOWN                   [CROSS]         Cross = Feeder

       [L3]                [R3]          D-pad Up = Arm up
      (stick)             (stick)        D-pad Down = Arm down
```

### Quick Reference

- **Square** is your main intake button - it automatically lowers the arm and runs the rollers
- **Circle** spits out game pieces if you grabbed the wrong one
- **L1** spins up the shooter
- **Cross** feeds pieces into the shooter (use with L1)

## File Structure

Here's where everything lives:

```
src/main/java/frc/robot/
├── Robot.java              # Main robot code (don't touch unless you know what you're doing)
├── RobotContainer.java     # All subsystems and button bindings
├── Constants.java          # All the numbers (CAN IDs, speeds, etc.)
│
├── commands/               # What the robot does
│   ├── RunMotorGroup1Command.java      # Feeder
│   ├── RunMotorGroup2ForwardCommand.java # Shooter forward
│   ├── RunMotorGroup2ReverseCommand.java # Shooter reverse
│   ├── RunIntakeCommand.java           # Intake rollers
│   ├── RunIntakeReverseCommand.java    # Eject
│   ├── AutoDeployIntakeCommand.java    # Auto intake (the smart one!)
│   ├── DeployUpCommand.java            # Arm up
│   └── DeployDownCommand.java          # Arm down
│
└── subsystems/             # The robot's "brains"
    ├── MotorGroup1Subsystem.java       # Feeder motors
    ├── MotorGroup2Subsystem.java       # Shooter motors
    ├── IntakeSubsystem.java            # Intake rollers
    ├── IntakeDeploySubsystem.java      # Intake arm
    ├── PowerManagementSubsystem.java   # Battery monitoring
    └── TelemetrySubsystem.java         # Dashboard stuff
```

## Changing Things

### Motor Speeds
Open `Constants.java` and look for:
- `kMotorGroup1PowerPercent` - Feeder speed (0-100%)
- `kMotorGroup2TargetRPM` - Shooter speed (RPM)
- `kIntakePowerPercent` - Intake roller speed (0-100%)

### CAN IDs
If you need to change motor CAN IDs, they're all in `Constants.java` under `MotorConstants`.

### Button Bindings
Want a different button layout? Edit `RobotContainer.java` in the `configureBindings()` method.

## Building and Deploying

1. Make sure you have WPILib 2026 installed
2. Connect to the robot's WiFi or plug in via USB
3. In VS Code, press `Ctrl+Shift+P` and type "Deploy Robot Code"
4. Wait for it to deploy (takes about 30 seconds)

## Troubleshooting

**Motors not spinning?**
- Check that the CAN IDs match what's in `Constants.java`
- Make sure the robot is enabled
- Check the battery voltage (should be above 12V)

**Controller not working?**
- Make sure it's plugged into port 0 on the driver station
- Check that it shows up in the Driver Station software

**Code won't deploy?**
- Make sure you're connected to the robot
- Try "Reset Robot Code" from the Driver Station
- Restart the roboRIO if all else fails

## Credits

- **Team 3020** - That's you guys!
- **Team 5805 (Alphabot)** - the sister team who shared their code
- **Author**: Baichen Yu

Good luck at competition!
