#!/bin/bash
# KVS Stream Log Viewer
# Color-coded log viewing for AWS Kinesis Video Streams
# Usage: ./kvs_log_viewer.sh
# Or: tail -f ~/.ros/log/kvs_stream.log | ./kvs_log_viewer.sh

LOG_FILE="$HOME/.ros/log/kvs_stream.log"

# Check if log file exists
if [ ! -f "$LOG_FILE" ]; then
    echo "Log file not found: $LOG_FILE"
    echo "Waiting for log file to be created..."
    while [ ! -f "$LOG_FILE" ]; do
        sleep 1
    done
    echo "Log file created. Starting viewer..."
fi

# Color codes
RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Function to colorize log lines
colorize_log() {
    while IFS= read -r line; do
        if [[ "$line" =~ ERROR|Error|error|FAILED|Failed|failed ]]; then
            echo -e "${RED}$line${NC}"
        elif [[ "$line" =~ WARNING|Warning|warning|WARN|Warn|warn ]]; then
            echo -e "${YELLOW}$line${NC}"
        elif [[ "$line" =~ STREAMING_STARTED|Started|started|SUCCESS|Success|success ]]; then
            echo -e "${GREEN}$line${NC}"
        elif [[ "$line" =~ INFO|Info|info ]]; then
            echo -e "${CYAN}$line${NC}"
        elif [[ "$line" =~ DEBUG|Debug|debug ]]; then
            echo -e "${BLUE}$line${NC}"
        else
            echo "$line"
        fi
    done
}

# Display header
echo "=========================================="
echo "KVS Stream Log Viewer"
echo "=========================================="
echo "Log file: $LOG_FILE"
echo "Press Ctrl+C to exit"
echo "=========================================="
echo ""

# Tail the log file with colorization
if [ -t 0 ]; then
    # Running as standalone script
    tail -f "$LOG_FILE" 2>/dev/null | colorize_log
else
    # Receiving input from pipe
    colorize_log
fi
