# Docker for ROS, Theory and Practice

This is the Docker image for [ROS, Theory and Practice, 深蓝学院](https://www.shenlanxueyuan.com/my/course/246) and [Robotics Software Engineer, Udacity](https://www.udacity.com/course/robotics-software-engineer--nd209).

---

## Install Docker & Docker-Compose

First, install `Docker` and `Docker-Compose`.

---

### Install Docker

Please follow [Docker Official Guide](https://docs.docker.com/engine/install/ubuntu/) and finish `Docker` installation

After installation `remember to add current user to docker group` so that docker can be used without `sudo`:

```bash
sudo usermod -aG docker $USER
```

---

### Install Docker-Compose

Please follow [Docker Compose Official Guide](https://docs.docker.com/compose/install/) and finish `Docker-Compose` installation.

---

## Launch Development Environment

---

### Configuration

Before launching the instance please change the config according to your local environment. The launch script can be found [here](docker-compose.yml).

```yaml
  volumes:
    # source code:
    - ${PWD}/workspace/assignments:/workspace/assignments
    # data:
    - ${PWD}/workspace/data:/workspace/data
  ports:
    # HTML5 VNC:
    - 40080:80
    # standard VNC client:
    - 45901:5901
    # supervisord admin:
    - 49001:9001
    # ROS master:
    - 11311:11311
```

---

### Up and Running 

#### Launch VNC Instance

```bash
# using docker-compose
docker-compose down && docker-compose up workspace-vnc-bionic-cpu
```

---

#### Health Check

Access supervisord monitor to make sure all processes have been started (default password `shenlanxueyuan`): http://[HOST_IP]:49001

![Supervisord Health Check](doc/01-supervisord-health-check.png)

---

#### VNC Access:

You can access the desktop with standard VNC client (default password is NONE)

##### VNC Client

On the system you can use 

* Remmina on Ubuntu
* TightVNC Viewer on Windows

to create a VNC session to connect to: http://[HOST_IP]:45901

![VNC through VNC Client](doc/02-vnc-access-with-vnc-client.png)

##### HTML5 VNC

Or interact with the environment directly inside your browser (Google Chrome is recommended)

1. Open Google Chrome and go to http://[HOST_IP]:40080
2. For the first connection the internal desktop service will be restarted to adjust to your current view portal size. **Just be patient**

After **view portal initialization** you will have the following environment available in your browser

![VNC through Web Browser](doc/03-vnc-access-with-browser.png)