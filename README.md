# FTC sim

Runs a team's **real** TeamCode OpModes on a PC against fake hardware, on a 3D BIOBUZZ (2026-27) field you
drive with a controller in the browser. It compiles against the real FTC SDK 12.0 jars (plus SolversLib and
Pedro), so code that works here is the same code that goes on the robot.

It lives inside a team repo as the `sim/` folder, a git submodule. The robot app's build never sees it.

## Use it in a team repo

```
git submodule add <this repo's URL> sim
```

Then, in the team repo:

- `sim.properties` picks the robot: `robot=starter` for the built-in starter bot. `host=localhost` keeps the page
  to that computer.
- A `runSim` task in the root `build.gradle` and a `.run/Run in Sim.run.xml` give Android Studio a **Run in Sim**
  button. The starter repo has both, copy them from there.
- Or from a terminal: `.\gradlew -p sim run`

It opens http://localhost:8080. Pick an OpMode, press INIT, then START, like the Driver Station. Pressing Run
again closes the old sim, and the open page reconnects to the new one.

It needs the Android SDK (it reads `local.properties`, which Android Studio writes), and internet the first time.

## What it checks for students

- **Device names** work like the real config: a name that isn't in the robot's config crashes with the SDK's own
  message, plus "Did you mean intake?" for typos.
- **Crashes** show the stack trace with a plain English `HINT` on top for the common ones (`Hints.java`).
- **Stuck code**: an OpMode stuck inside `loop()` (a `while` loop, usually) for 3 s gets a warning with the file
  and line. A LinearOpMode that ignores STOP gets one too, and its motor writes stop counting.
- **Moving during INIT** shows a warning (marked as allowed in autos).
- **Motor directions**: motors are mounted the way a real robot's are, so the left side needs `REVERSE`.
- **Java 8**: TeamCode compiles as Java 8, like the robot, so `var` fails here too.

## Robots

A robot is a `SimRobot` subclass: its hardware config, drivetrain numbers, collision outline, and a `step()` that
makes its mechanisms do things (spin a flywheel, pick up and shoot balls). Plus a 3D model in `web/robots/`.

- `StarterRobot` is built in: mecanum drive, `intake`, `shooter`, `gate`. The starter repo's `START_HERE.md`
  describes it for students.
- A team's own robot lives in the team repo, so its code and CAD stay there: the class in `sim-robot/java/`
  (it can read the team's constants), the model in `sim-web/robots/`, and `robot=<full class name>` in
  `sim.properties`. Files in `sim-web/` win over the sim's own `web/`.

Models: every moving part in the robot's json has a hinge (`pivot` + `axis`) and a driver (a servo position, a
motor's rpm). Parts are boxes and cylinders, or a `.glb` file from the CAD (`cad/` has the export scripts).

## Working on the sim itself

On its own (not inside a team repo) it compiles `example/TeamCode` instead, which has an answer-key TeleOp for
the starter bot.

```
.\gradlew run
.\gradlew test
```

The tests drive the starter bot like a student would (drive, intake, shoot) and check each beginner mistake gets
its message. They only run on their own, not inside a team repo.

## How it works

- `build.gradle` compiles the team's `TeamCode/src/main/java` as Java 8 against the SDK jars, then the sim on top.
- `SimHardwareMap` hands out fake motors, servos, the OTOS, analog inputs, the Limelight and the battery.
- `SimOpModeRunner` drives INIT/START/STOP the same way the Robot Controller app does.
- `SimPhysics` is the shared physics: per-wheel mecanum forces with a motor curve and traction limit, the
  battery, the balls a robot holds, and what a Limelight sees. `SimCollisions` is the ode4j world: walls,
  field shapes from the CAD, real balls, and hinged Hives that tip.
- Every number marked TUNE is a guess.

## The field

Shapes come from the field CAD (`cad/field_colliders.py`). Open http://localhost:8080/?colliders to see them.
Balls start as the manual's section 10.3.1 says. Hives hold either end and tip when enough weight lands in the
upward cell. The panel shows tips and balls in cells (20 and 2 points, manual table 10-2).

## The website

`site.bat` puts the sim online behind invite codes (`site/invite_codes.txt`, not in git, one per line:
`CODE name`). Each person gets their own sim process, up to 6. The page loads three.js from a CDN, so it needs
internet.

The Settings button changes a robot's real constants live (a robot adds them with `SimSettings.slider` and
`toggle`). Sounds are made in the browser from the sim's state (`web/sounds.js`), off until M.
