#!/usr/bin/env bash
iperf3 -s 192.168.50.155 -p 3000 -J >>log_$1_$2_server.json
