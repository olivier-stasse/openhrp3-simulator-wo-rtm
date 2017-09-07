#!/bin/bash
ps aux | grep openhrp-model-loader | awk '{print $2}' | xargs kill 
ps aux | grep openhrp-collision-detector | awk '{print $2}' | xargs kill
ps aux | grep openhrp-aist-dynamics-simulator | awk '{print $2}' | xargs kill
ps aux | grep gepetto-online-viewer | awk '{print $2}' | xargs kill
ps aux | grep controller-hrp2 | awk '{print $2}' | xargs kill
ps aux | grep hrpsys-viewer | awk '{print $2}' | xargs kill
ps aux | grep controller-sample | awk '{print $2}' | xargs kill
ps aux | grep bin/tilt-plate-controller | awk '{print $2}' | xargs kill

