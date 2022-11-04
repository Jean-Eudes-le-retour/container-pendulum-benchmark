"""Controller program to manage the benchmark.

It manages the perturbation and evaluates the performance of the user
controller.
"""

from controller import Supervisor
import os
import random
import sys
import time as timeLib

# set to True if you want to generate an animation of your controller to ./storage/local
RECORD_ANIMATION = False

import recorder.recorder as rec

def benchmarkPerformance(message, robot):
    benchmark_name = message.split(':')[1]
    benchmark_performance_string = message.split(':')[3]
    print(benchmark_name + ' Benchmark complete! Your performance was ' + benchmark_performance_string)

# Get random generator seed value from 'controllerArgs' field
seed = 1
if len(sys.argv) > 1 and sys.argv[1].startswith('seed='):
    seed = int(sys.argv[1].split('=')[1])

robot = Supervisor()

timestep = int(robot.getBasicTimeStep())

jointParameters = robot.getFromDef("PENDULUM_PARAMETERS")
positionField = jointParameters.getField("position")

# emitter needed for the physics plugin?
emitter = robot.getDevice("emitter")
time = 0
force = 0
forceStep = 800
random.seed(seed)
running = True

if RECORD_ANIMATION:
    # Recorder code: wait for the controller to connect and start the animation
    rec.animation_start_and_connection_wait(robot)
    step_max = 1000 * rec.MAX_DURATION / timestep
    step_counter = 0

while robot.step(timestep) != -1 and running:
    if running:
        time = robot.getTime()
        robot.wwiSendText("time:%-24.3f" % time)
        robot.wwiSendText("force:%.2f" % force)

        # Detect status of inverted pendulum
        position = positionField.getSFFloat()
        if position < -1.58 or position > 1.58:
            # stop
            running = False
            name = "Inverted Pendulum"
            performance = str(time)
            performanceString = rec.time_convert(time)
            message = 'success:' + name + ':' + performance + ':' + performanceString
            robot.wwiSendText(message)
        else:
            if forceStep <= 0:
                forceStep = 800 + random.randint(0, 400)
                force = force + 0.02
                toSend = "%.2lf %d" % (force, seed)
                if sys.version_info.major > 2:
                    toSend = bytes(toSend, "utf-8")
                emitter.send(toSend)
            else:
                forceStep = forceStep - 1
    if RECORD_ANIMATION:
        # Stops the simulation if the controller takes too much time
        step_counter += 1
        if step_counter >= step_max:
            break

if RECORD_ANIMATION:
    # Write performance to file, stop recording and close Webots
    rec.record_performance(running, time)
    rec.animation_stop(robot, timestep)
    robot.simulationQuit(0)
else:
    benchmarkPerformance(message, robot)

robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)