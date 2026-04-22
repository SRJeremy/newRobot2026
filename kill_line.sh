#!/bin/bash
declare -i line_pid=$(pidof -s Line_Process)
kill $line_pid

