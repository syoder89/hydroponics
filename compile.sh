#!/bin/bash

particle compile argon . --saveTo hydro.bin --target 1.1.1
particle flash Hydroponics hydro.bin
