import os
from flask import Flask
import json

# Refer to the following link for information about how to pass arguments to a Flask APP
# https://stackoverflow.com/questions/73635412/flask-run-with-argument
def create_app(ws_path):
    print("Workspace Path: ", ws_path)
    os.environ["PATH"] = "{}:{}".format(os.environ["PATH"], ws_path)
    os.environ["WS_PATH"] = ws_path
    app = Flask(__name__)

    @app.route("/")
    def hello_world():
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
        if(status == 0):
            return json.dumps({'success':True}), 200, {'ContentType':'application/json'}
        else:
            return json.dumps({'success':False}), 500, {'ContentType':'application/json'}

    return app
