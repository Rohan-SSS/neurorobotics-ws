
import os
from flask import Flask
import json
# Install psutil
import psutil
import time

def checkIfProcessRunning(processName):
    '''
    Check if there is any running process that contains the given name processName.
    '''
    #Iterate over the all the running process
    for proc in psutil.process_iter():
        try:
            # Check if process name contains the given name string.
            if processName.lower() in proc.name().lower():
                return True
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return False;

# Refer to the following link for information about how to pass arguments to a Flask APP
# https://stackoverflow.com/questions/73635412/flask-run-with-argument
def create_app(ws_path):
    print("Workspace Path: ", ws_path)
    # Need to set the following values in .bashrc to be able to use djinn in general as well.
    os.environ["PATH"] = "{}:{}".format(os.environ["PATH"], ws_path)
    os.environ["WS_PATH"] = ws_path
    # os.environ["DJINN_MODE"] = "automation"
    app = Flask(__name__)

    @app.route("/")
    def hello_world():
        status = os.system ("djinn")
        return "<p>Welcome to Oracle for SkyNet. Oracle Provides a Web API to interface with djinn!</p>"

    @app.route("/setup/<arg>")
    def setup_sitl(arg):
        status = os.system('djinn setup docker {}'.format(arg))
        if(status == 0):
            return json.dumps({'success':True}), 200, {'ContentType':'application/json'}
        else:
            return json.dumps({'success':False}), 500, {'ContentType':'application/json'}

    @app.route("/init/<arg>")
    def init_sitl(arg):
        status = os.system('djinn init {}'.format(arg))
        if(status == 0):
            return json.dumps({'success':True}), 200, {'ContentType':'application/json'}
        else:
            return json.dumps({'success':False}), 500, {'ContentType':'application/json'}
     
    @app.route("/start/<arg>/<env>")
    def start_sitl(arg, env):
        print("Starting Process: ", arg)
        print("Starting Environment: ", env)
        status = os.system('djinn start {} {}'.format(arg, env))
        time.sleep(15)
        if status != 0:
            return json.dumps({'success':False}), 500, {'ContentType':'application/json'}
        else:
            status = True
        print("Spawn Status: ", status)

        if arg == "sitl":
            status_skynet = checkIfProcessRunning("skynet")
            print("SkyNet Status: ", status_skynet)
            status_px4 = checkIfProcessRunning("px4")
            print("PX4 Status: ", status_px4)
            status = status and status_px4 and status_skynet
        elif arg == "airsim":
            status_airsim_lib = checkIfProcessRunning("airsim_lib")
            print("Airsim Lib Status: ", status_airsim_lib)
            status = status and status_airsim_lib
        status_env = checkIfProcessRunning(env) 
        print("Environment Status: ", status_env)
        status = status and status_env
        if(status):
            return json.dumps({'success':True}), 200, {'ContentType':'application/json'}
        else:
            return json.dumps({'success':False}), 500, {'ContentType':'application/json'}

    @app.route("/check/<process>")
    def check_process(process):
        print("Checking for existence of process by name: ", process)
        status = checkIfProcessRunning(process)
        return json.dumps({'success':True, 'processName': process, "processRunning": status}), 200, {'ContentType':'application/json'}

    @app.route("/kill/<process>")
    def kill_process(process):
        status = os.system('djinn kill {}'.format(process))
        if status == 0:
            return json.dumps({'success':True}), 200, {'ContentType':'application/json'}
        else:
            return json.dumps({'success':False}), 500, {'ContentType':'application/json'}

    return app
