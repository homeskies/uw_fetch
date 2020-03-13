# web_teleop

A web application to teleoperate the robot. Leverages [librosjs](http://wiki.ros.org/roslibjs) to forward commands
to /cmd_vel and to a custom node which forwards other commands to the robot. Currently, this node is tightly coupled
with our `fetch_api`.

* Compatible with [RWS](https://github.com/hcrlab/rws)

## Usage

First, make sure you have the [Polymer CLI](https://www.npmjs.com/package/polymer-cli) installed. You'll also need to install bower.

    npm install -g bower
    npm install -g polymer-cli

Then, to build, run the following in the frontend folder:

    polymer build
    
This will create builds of your application in the `build/` directory, optimized to be served in production. You can then serve the built versions by giving `polymer serve` a folder to serve from:

The application requires

## Running Tests

    polymer test

Your application is already set up to be tested via [web-component-tester](https://github.com/Polymer/web-component-tester). Run `polymer test` to run your application's test suite locally.
