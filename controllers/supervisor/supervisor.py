"""Controller program to manage the benchmark.

It manages the perturbation and evaluates the performance of the user
controller.
"""

from controller import Supervisor
import os
import random
import sys

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
            message = f'success:{name}:{time}'
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

# Performance output used by automated CI script
CI = os.environ.get("CI")
if CI:
    print(f"performance_line:{time}")
else:
    benchmarkPerformance(message, robot)

robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)